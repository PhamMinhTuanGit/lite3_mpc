#!/usr/bin/env python3
"""Run the read-only GMO/GRF MuJoCo validation matrix sequentially."""

import argparse
import subprocess
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parent.parent
RUNNER = REPO / "tools" / "run_cmpc_payload_test.py"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--dry", action="store_true")
    parser.add_argument("--scenarios", nargs="+",
                        choices=("nominal", "payload", "early-contact"),
                        default=("nominal", "payload", "early-contact"))
    args = parser.parse_args()

    for scenario in args.scenarios:
        command = [sys.executable, str(RUNNER), "--scenario", scenario]
        if args.viewer:
            command.append("--viewer")
        if args.dry:
            command.append("--dry")
        print(f"\n=== GMO validation scenario: {scenario} ===", flush=True)
        completed = subprocess.run(command, cwd=REPO, check=False)
        if completed.returncode != 0:
            raise SystemExit(
                f"scenario {scenario} failed with exit code {completed.returncode}")


if __name__ == "__main__":
    main()
