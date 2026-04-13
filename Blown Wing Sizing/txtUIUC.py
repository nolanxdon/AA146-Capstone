"""
UIUC Static Prop Digitization Script (Expanded Version)
Author: Nolan

This version:
- Generates all CT0 / CP0 vs RPM txt files
- Writes them to UIUCProp/
- Immediately reads each file back
- Prints full contents to terminal

This allows you to run once and copy/verify each file.
"""

import numpy as np
import os

# ============================================================
# SETTINGS
# ============================================================

OUTPUT_DIR = "UIUCSmall"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================================================
# FILE WRITER
# ============================================================

def write_prop_file(name, rpm_start, rpm_end, rpm_step, ct_start, ct_end, cp_start, cp_end):
    rpm_vals = np.arange(rpm_start, rpm_end + 1e-9, rpm_step)
    ct_vals = np.linspace(ct_start, ct_end, len(rpm_vals))
    cp_vals = np.linspace(cp_start, cp_end, len(rpm_vals))

    path = os.path.join(OUTPUT_DIR, f"{name}_CT_CP_vs_RPM.txt")

    with open(path, "w") as f:
        f.write("RPM\tCT0\tCP0\n")
        for r, ct, cp in zip(rpm_vals, ct_vals, cp_vals):
            f.write(f"{int(r)}\t{ct:.5f}\t{cp:.5f}\n")

    print(f"\n=== GENERATED: {path} ===")

    # Print file contents immediately
    with open(path, "r") as f:
        print(f.read())

# ============================================================
# PROP DEFINITIONS
# ============================================================

write_prop_file("APC_FreeFlight_4.2x4",1500,10000,500,0.122,0.130,0.135,0.108)
write_prop_file("APC_Sport_4.2x2",2500,15200,800,0.065,0.090,0.050,0.042)
write_prop_file("DA4002_5in_-10deg",1500,10500,600,0.060,0.085,0.040,0.036)
write_prop_file("DA4002_5in_-5deg",1500,8000,400,0.085,0.102,0.063,0.058)
write_prop_file("DA4002_5in_0deg",1500,7400,400,0.115,0.125,0.098,0.088)
write_prop_file("DA4002_5in_+5deg",1500,7000,400,0.140,0.148,0.148,0.145)
write_prop_file("DA4022_5in_3blade",1500,7000,300,0.200,0.215,0.155,0.130)
write_prop_file("DA4022_5in_4blade",1500,6000,300,0.235,0.245,0.190,0.165)
write_prop_file("DA4022_5in_2blade",2100,7200,300,0.150,0.158,0.105,0.092)
write_prop_file("EFlite_130x70",1500,9500,500,0.085,0.112,0.050,0.045)
write_prop_file("GWS_DD_3x3",3000,15000,1000,0.185,0.198,0.158,0.140)
write_prop_file("MicroInvent_4.3x3.5_3blade",2000,9500,500,0.140,0.175,0.088,0.108)
write_prop_file("Union_U80_80x50mm",3000,18000,1000,0.130,0.159,0.092,0.082)

print("\nAll expanded prop coefficient files generated and printed successfully.")
