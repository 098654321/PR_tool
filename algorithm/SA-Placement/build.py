"""Build isolated, instrumented copies; never write to the repository source tree."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


def replace_exact(text, before, after, count=1):
    actual = text.count(before)
    if actual != count:
        raise RuntimeError(f"Source instrumentation anchor changed: expected {count}, got {actual}: {before!r}")
    return text.replace(before, after)


def instrument_sa(text):
    text = '#include "experiment_support.hh"\n' + text
    replacements = [
        ("for (std::usize n = 0; n < budget.solve_num; ++n) {",
         "for (std::usize n = 0; n < budget.solve_num; ++n) {\n                ++sa_experiment::stats.attempts;", 1),
        ('debug::debug("Swap two top-level chip instance");',
         '++sa_experiment::stats.swap_attempts;\n                    debug::debug("Swap two top-level chip instance");', 1),
        ('debug::debug("Move one top-level chip instance to another TOB");',
         '++sa_experiment::stats.move_attempts;\n                    debug::debug("Move one top-level chip instance to another TOB");', 1),
        ("if (deltaCost <= 0) {", "if (deltaCost <= 0) {\n                        ++sa_experiment::stats.accepted_nonworse;", 2),
        ("if (random_f64() <= std::exp(exp_x)) {",
         "if (random_f64() <= std::exp(exp_x)) {\n                            ++sa_experiment::stats.accepted_uphill;", 2),
        ('debug::debug("Reject movement");', '++sa_experiment::stats.rejected;\n                            debug::debug("Reject movement");', 2),
        ("temperature = this->calculate_next_temperature(temperature, iteration);",
         "sa_experiment::temperature_end(iteration + 1, temperature, total_cost, best_cost, no_improvement_count);\n"
         "            temperature = this->calculate_next_temperature(temperature, iteration);", 1),
        ("this->restore_placement(topdies, best_solution);",
         "sa_experiment::finish(temperature, no_improvement_count, iteration);\n"
         "        this->restore_placement(topdies, best_solution);", 1),
        ("std::random_device rd;\n        std::mt19937_64 gen(rd());\n\n"
         "        std::uniform_real_distribution<double> dis(0.0, 1.0);\n        return dis(gen);",
         "return sa_experiment::random_real();", 1),
    ]
    for before, after, count in replacements:
        text = replace_exact(text, before, after, count)
    return text


def source_inputs():
    return sorted(p for group in ("algo", "circuit", "hardware", "global", "parse", "serde")
                  for p in (ROOT / "source" / group).rglob("*")
                  if p.suffix in (".cc", ".hh", ".h") and "sat_ilp" not in p.parts
                  and p.name != "sat_backend.cc")


def build(widths, jobs, compiler, log_dir):
    command = shlex.split(compiler)
    version = subprocess.run(command + ["--version"], check=True, text=True, capture_output=True).stdout
    inputs = source_inputs()
    digest = hashlib.sha256()
    for path in inputs + [HERE / "main.cc", HERE / "experiment_support.hh", Path(__file__), HERE / "run_experiments.py"]:
        digest.update(str(path.relative_to(ROOT)).encode())
        digest.update(path.read_bytes())
    digest.update(json.dumps(command).encode())
    digest.update(version.encode())
    fingerprint = digest.hexdigest()
    build_dir = HERE / ".build" / fingerprint[:16]
    source = build_dir / "source"
    source.mkdir(parents=True, exist_ok=True)
    # Use real copies, not symlinks: quoted includes must see the same width as
    # every translation unit, and generated modifications must stay local.
    for path in inputs:
        dest = source / path.relative_to(ROOT / "source")
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, dest)
    sa = source / "algo/placer/sa/saplacestrategy.cc"
    sa.write_text(instrument_sa((ROOT / "source/algo/placer/sa/saplacestrategy.cc").read_text()))
    header = source / "hardware/interposer.hh"
    text, count = re.subn(r"COB_ARRAY_WIDTH\s*=\s*\d+", "COB_ARRAY_WIDTH = SA_EXPERIMENT_COB_WIDTH", header.read_text())
    # On a cached build the header is already instrumented.
    if count != 1 and "COB_ARRAY_WIDTH = SA_EXPERIMENT_COB_WIDTH" not in text:
        raise RuntimeError("Cannot instrument COB width")
    header.write_text(text)
    interposer = source / "hardware/interposer.cc"
    text = (ROOT / "source/hardware/interposer.cc").read_text()
    text = replace_exact(text, "return idle_tobs[random_i64(0, idle_tobs.size() - 1)];",
                         "std::sort(idle_tobs.begin(), idle_tobs.end(), [](auto* a, auto* b) {\n"
                         "            const auto x = a->coord(), y = b->coord();\n"
                         "            return std::tie(x.row, x.col) < std::tie(y.row, y.col);\n"
                         "        });\n        return idle_tobs[random_i64(0, idle_tobs.size() - 1)];")
    interposer.write_text("#include <algorithm>\n#include <tuple>\n" + text)
    (source / "global/utility/random.cc").write_text(
        '#include "experiment_support.hh"\n#include <utility/random.hh>\n'
        'namespace PR_tool {\n'
        'auto random() -> double { return sa_experiment::random_real(); }\n'
        'auto random_i64(std::i64 min, std::i64 max) -> std::i64 {\n'
        '    return sa_experiment::random_integer(min, max);\n}\n}\n')
    units = sorted(source.rglob("*.cc")) + [HERE / "main.cc"]
    binaries = {}
    for width in sorted(widths):
        folder = build_dir / f"width{width}"
        folder.mkdir(exist_ok=True)
        binary = folder / "sa_experiment"
        binaries[width] = binary
        log_file = log_dir / f"build-width{width}.log"
        with log_file.open("w") as log:
            log.write(version + f"\nsource_fingerprint={fingerprint}\n")
            if binary.exists():
                log.write(f"Using cached binary: {binary}\n")
                continue
            flags = ["-std=c++20", "-O2", "-DNDEBUG", "-DPR_TOOL_HAS_SAT_ROUTER=0",
                     f"-DSA_EXPERIMENT_COB_WIDTH={width}", "-pthread",
                     "-I", str(source), "-I", str(source / "global"), "-I", str(HERE)]

            def compile_one(item):
                index, unit = item
                obj = folder / f"{index:03d}.o"
                if obj.exists():
                    return obj, ""
                temporary = obj.with_suffix(".tmp.o")
                cmd = command + flags + ["-c", str(unit), "-o", str(temporary)]
                result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                output = shlex.join(cmd) + "\n" + result.stdout
                if result.returncode:
                    return None, output
                temporary.replace(obj)
                return obj, output

            objects, failed = [], False
            with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
                for obj, output in pool.map(compile_one, enumerate(units)):
                    log.write(output)
                    log.flush()
                    failed |= obj is None
                    if obj is not None:
                        objects.append(obj)
            if failed:
                raise RuntimeError(f"Compilation failed; see {log_file}")
            temporary_binary = binary.with_suffix(".tmp")
            cmd = command + [str(p) for p in objects] + ["-pthread", "-o", str(temporary_binary)]
            result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            log.write(shlex.join(cmd) + "\n" + result.stdout)
            if result.returncode:
                raise RuntimeError(f"Link failed; see {log_file}")
            temporary_binary.replace(binary)
    return binaries, {"source_fingerprint": fingerprint, "compiler": command,
                      "compiler_version": version, "compile_flags": "C++20 -O2 -DNDEBUG -pthread; SAT disabled",
                      "source_files": len(inputs)}
