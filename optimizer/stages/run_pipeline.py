from __future__ import annotations

from optimizer.stages.stage1_screen import run_stage1
from optimizer.stages.stage2_prop_span import run_stage2
from optimizer.stages.stage3_aerosandbox import run_stage3
from optimizer.stages.validate_outputs import validate_outputs


def run_pipeline() -> None:
    run_stage1()
    run_stage2()
    run_stage3()
    issues = validate_outputs()
    if issues:
        print("Validation issues:")
        for issue in issues:
            print(f"  - {issue}")
        raise SystemExit(1)
    print("Validation: outputs are internally consistent.")


if __name__ == "__main__":
    run_pipeline()
