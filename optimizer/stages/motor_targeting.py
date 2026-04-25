from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.motor_targeting import run_motor_targeting


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a motor/ESC target sheet for a selected ranked propulsion concept.",
    )
    parser.add_argument("--rank", type=int, default=6, help="Stage 3 rank to evaluate. Defaults to 6.")
    parser.add_argument("--blade-count", type=int, default=3, help="Blade count metadata. Defaults to 3.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/motor_targeting"),
        help="Root directory for the motor-target artifacts.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_motor_targeting(
        rank=args.rank,
        blade_count_metadata=args.blade_count,
        output_root=args.output_root,
    )
    print("Motor targeting")
    print(f"  Summary CSV:             {output.summary_csv}")
    print(f"  Summary markdown:        {output.summary_md}")
    print(f"  Candidate CSV:           {output.candidate_csv}")
    print(f"  Operating-point plot:    {output.operating_plot}")


if __name__ == "__main__":
    main()
