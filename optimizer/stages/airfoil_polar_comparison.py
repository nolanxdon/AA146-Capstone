from __future__ import annotations

import argparse
from pathlib import Path

from optimizer.core.airfoil_polar_comparison import (
    AirfoilPolarComparisonConfig,
    run_airfoil_polar_comparison,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare blown-wing section polars for multiple airfoils and export legacy-style polar files."
    )
    parser.add_argument(
        "--airfoils",
        nargs="+",
        default=["s1210", "naca2412", "e423"],
        help="Airfoil names recognized by AeroSandbox / NeuralFoil.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/airfoil_comparison"),
        help="Directory for plots, CSVs, and markdown summary.",
    )
    args = parser.parse_args()

    config = AirfoilPolarComparisonConfig(
        airfoils=tuple(args.airfoils),
        output_root=args.output_root,
        legacy_output_root=args.output_root / "legacy_style_polars",
    )
    outputs = run_airfoil_polar_comparison(config)

    print("Airfoil blowing comparison")
    print(f"  Airfoils:                {', '.join(config.airfoils)}")
    print(f"  Output root:             {config.output_root}")
    print(f"  Curve CSV:               {outputs['curve_csv']}")
    print(f"  Metric CSV:              {outputs['metric_csv']}")
    print(f"  Cross-airfoil summary:   {outputs['comparison_png']}")
    print(f"  Markdown summary:        {outputs['summary_md']}")
    for airfoil_name, artifacts in outputs["airfoil_artifacts"].items():
        print(
            f"  {_display_name(config, airfoil_name)}: "
            f"cd-grid={artifacts['response_grid_cd_png']} | "
            f"cx-grid={artifacts['response_grid_cx_png']} | "
            f"legacy-count={len(artifacts['legacy_paths'])}"
        )


def _display_name(config: AirfoilPolarComparisonConfig, airfoil_name: str) -> str:
    return config.display_names.get(airfoil_name.lower(), airfoil_name)


if __name__ == "__main__":
    main()
