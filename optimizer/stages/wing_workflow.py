from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.wing_workflow import run_wing_workflow


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the unified blown-wing workflow: airfoil front-end, DAE51 controls, motor height, and speed sweep.",
    )
    parser.add_argument("--rank", type=int, default=6, help="Stage 3 rank to evaluate. Defaults to 6.")
    parser.add_argument("--blade-count", type=int, default=3, help="Blade count metadata. Defaults to 3.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/wing_workflow"),
        help="Root directory for the unified workflow outputs.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_wing_workflow(
        rank=args.rank,
        blade_count_metadata=args.blade_count,
        output_root=args.output_root,
    )
    print("Unified wing workflow")
    print(f"  Output root:             {output.output_root}")
    print(f"  Summary CSV:             {output.summary_csv}")
    print(f"  Summary markdown:        {output.summary_md}")


if __name__ == "__main__":
    main()
