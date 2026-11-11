"""Keep cancellation effective inside WSL even when the Windows GUI exits."""

import argparse
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cancel-file", type=Path, required=True)
    parser.add_argument("--owner-file", type=Path, required=True)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    process = subprocess.Popen(command)
    deadline = time.monotonic() + args.timeout
    try:
        while process.poll() is None:
            owner_lost = time.time() - args.owner_file.stat().st_mtime > 20
            if args.cancel_file.exists() or owner_lost or time.monotonic() > deadline:
                raise RuntimeError("External task cancelled, timed out, or GUI worker disconnected")
            time.sleep(0.5)
        return process.returncode
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
