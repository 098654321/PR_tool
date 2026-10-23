"""CLI budget contract. Run with a CLI binary and a width-compatible config."""
import pathlib
import subprocess
import sys
import tempfile
import time


def main():
    binary, config = (str(pathlib.Path(p).resolve()) for p in sys.argv[1:3])
    invalid = [[], ["0"], ["-1"], ["nan"], ["inf"], ["abc"], ["1s"]]
    with tempfile.TemporaryDirectory(prefix="rrr-cli-") as temporary:
        root = pathlib.Path(temporary)
        for i, value in enumerate(invalid):
            output = root / str(i)
            args = [binary, config, "-o", str(output), "--time-limit", *value]
            result = subprocess.run(args, capture_output=True, timeout=10)
            assert result.returncode != 0, f"accepted invalid budget: {value}"
            assert not (output / "regnamecontrolbit_4part").exists()
        output = root / "expired"
        start = time.monotonic()
        result = subprocess.run(
            [binary, config, "-o", str(output), "--time-limit", "0.000001"],
            capture_output=True, timeout=10,
        )
        elapsed = time.monotonic() - start
        assert result.returncode != 0, "expired budget reported success"
        assert not (output / "regnamecontrolbit_4part").exists(), "partial controlbits published"
        log = (output / "debug.log").read_text()
        assert "no complete legal solution" in log
        assert elapsed < 5, f"seconds budget ignored: {elapsed}"
    print("RRR CLI budget validation PASS")


if __name__ == "__main__":
    main()
