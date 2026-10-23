"""Build isolated, instrumented copies; never write to the repository source tree."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


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
    for path in inputs + sorted(p for p in HERE.iterdir() if p.suffix in (".py", ".hh", ".cc")):
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
    from instrument import instrument
    instrument(source)
    units = sorted(source.rglob("*.cc")) + [HERE / name for name in ("main.cc", "observation.cc", "self_test.cc")]
    binaries = {}
    for width in sorted(widths):
        folder = build_dir / f"width{width}"
        folder.mkdir(exist_ok=True)
        binary = folder / "order_experiment"
        binaries[width] = binary
        log_file = log_dir / f"build-width{width}.log"
        with log_file.open("w") as log:
            log.write(version + f"\nsource_fingerprint={fingerprint}\n")
            if binary.exists():
                log.write(f"Using cached binary: {binary}\n")
                continue
            flags = ["-std=c++20", "-O2", "-DNDEBUG", "-DPR_TOOL_HAS_SAT_ROUTER=0",
                     f"-DORDER_EXPERIMENT_COB_WIDTH={width}", "-pthread",
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
