from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.wing_speed_sweep import run_wing_speed_sweep


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a frozen-geometry speed sweep for the selected blown-wing configuration.",
    )
    parser.add_argument("--rank", type=int, default=6, help="Stage 3 rank to evaluate. Defaults to 6.")
    parser.add_argument("--blade-count", type=int, default=3, help="Blade count metadata. Defaults to 3.")
    parser.add_argument("--airfoil", type=str, default="dae51", help="Airfoil name to evaluate. Defaults to dae51.")
    parser.add_argument(
        "--prop-drop-chord",
        type=float,
        default=0.12,
        help="Motor vertical drop as a fraction of chord for the frozen configuration.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/wing_speed_sweep"),
        help="Root directory for the speed-sweep artifacts.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_wing_speed_sweep(
        rank=args.rank,
        blade_count_metadata=args.blade_count,
        airfoil_name=args.airfoil,
        prop_drop_fraction_of_chord=args.prop_drop_chord,
        output_root=args.output_root,
    )
    print("Frozen-geometry speed sweep")
    print(f"  Airfoil:                 {args.airfoil}")
    print(f"  Summary CSV:             {output.summary_csv}")
    print(f"  Summary markdown:        {output.summary_md}")
    print(f"  Curve CSV:               {output.curve_csv}")
    print(f"  Performance plot:        {output.performance_plot}")
    print(f"  Operating plot:          {output.operating_plot}")
    print(f"  Selected CL curves:      {output.selected_curve_plot}")


if __name__ == "__main__":
    main()
