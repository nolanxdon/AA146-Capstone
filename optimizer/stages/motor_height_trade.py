from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.motor_height_trade import run_motor_height_trade


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Trade study for propeller vertical drop relative to the wing/flap system.",
    )
    parser.add_argument("--rank", type=int, default=6, help="Stage 3 rank to evaluate. Defaults to 6.")
    parser.add_argument("--blade-count", type=int, default=3, help="Blade count metadata. Defaults to 3.")
    parser.add_argument("--airfoil", type=str, default="s1210", help="Airfoil name to evaluate. Defaults to s1210.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/motor_height_trade"),
        help="Root directory for the trade-study artifacts.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_motor_height_trade(
        rank=args.rank,
        blade_count_metadata=args.blade_count,
        airfoil_name=args.airfoil,
        output_root=args.output_root,
    )
    print("Motor height trade study")
    print(f"  Airfoil:                 {args.airfoil}")
    print(f"  Summary CSV:             {output.summary_csv}")
    print(f"  Summary markdown:        {output.summary_md}")
    print(f"  Overlay curve CSV:       {output.overlay_curve_csv}")
    print(f"  Metric plot:             {output.metric_plot}")
    print(f"  Whole-wing CL overlay:   {output.overlay_plot}")
    print(f"  Geometry plot:           {output.geometry_plot}")


if __name__ == "__main__":
    main()
