from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.control_surface_sizing import run_rectangular_control_surface_sizing


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Size rectangular-wing slotted flaps and ailerons for a selected ranked propulsion concept.",
    )
    parser.add_argument(
        "--rank",
        type=int,
        default=6,
        help="Stage 3 rank to size control surfaces for. Defaults to 6.",
    )
    parser.add_argument(
        "--blade-count",
        type=int,
        default=3,
        help="Blade count metadata for the selected prop concept. Defaults to 3.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/control_surface_sizing"),
        help="Root directory for the sizing artifacts.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output = run_rectangular_control_surface_sizing(
        rank=args.rank,
        blade_count_metadata=args.blade_count,
        output_root=args.output_root,
    )

    print("Rectangular-wing control surface sizing")
    print(f"  Rank:                    {output.concept.rank}")
    print(
        f"  Prop concept:            {output.concept.n_props} x "
        f"{output.concept.prop_diameter_in:.1f} x {output.concept.prop_pitch_in:.1f} in "
        f"({output.concept.prop_family}, {output.concept.blade_count_metadata}-blade metadata)"
    )
    print(
        f"  Recommended flap:        span={output.flap.flap_span_fraction:.2f} semispan | "
        f"cf/c={output.flap.flap_chord_fraction:.2f} | "
        f"deflection={output.flap.flap_deflection_deg:.1f} deg"
    )
    print(
        f"  Recommended aileron:     span={output.aileron.aileron_span_fraction:.2f} semispan | "
        f"cf/c={output.aileron.aileron_chord_fraction:.2f} | "
        f"roll rate @ 14 deg={output.aileron.roll_rate_at_nominal_degps:.1f} deg/s"
    )
    print(f"  Summary markdown:        {output.summary_md}")
    print(f"  Summary CSV:             {output.summary_csv}")
    print(f"  Flap sweep CSV:          {output.flap_sweep_csv}")
    print(f"  Aileron sweep CSV:       {output.aileron_sweep_csv}")
    print(f"  Layout plot:             {output.layout_plot}")
    print(f"  Flap heatmap:            {output.flap_heatmap_plot}")
    print(f"  Flap curves:             {output.flap_curves_plot}")
    print(f"  Flap section polars:     {output.flap_section_polar_plot}")
    print(f"  Total CL curve CSV:      {output.total_cl_curve_csv}")
    print(f"  Total CL curve plot:     {output.total_cl_curve_plot}")
    print(f"  Aileron curves:          {output.aileron_curves_plot}")


if __name__ == "__main__":
    main()
