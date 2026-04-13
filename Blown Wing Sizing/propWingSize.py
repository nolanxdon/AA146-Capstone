"""
Blown-wing / distributed-prop first-pass optimizer
Author: Nolan POV version

What changed in this version:
- I use a TWO-ZONE blown-wing model for both lift and drag:
    * unblown wing section uses q_inf = 0.5*rho*V_inf^2
    * blown wing section uses q_blow = 0.5*rho*V_eff^2
- I no longer compute drag from one giant freestream-referenced CL for the whole wing.
- I solve lift + thrust in a coupled way:
    * choose geometry + prop + motor count
    * iterate on RPM
    * RPM -> thrust -> V_eff
    * V_eff -> blown CLmax(Re)
    * check lift balance
    * compute two-zone drag
    * require thrust >= drag * margin
- I keep failure reason counts so I can see why 2 m designs are not converging.
- I do NOT cap blown-wing CLmax.
- I keep my Reynolds folder and prop txt folder structure.

Folder structure expected:
.
├── propWingSize.py
├── S1210/
│   ├── *.txt   <-- FLOW5 / XFOIL / XFLR5 style polars
└── UIUCSmall/
    ├── *.txt   <-- RPM, CT0, CP0
"""

import os
import re
import glob
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import numpy as np


# ============================================================
# USER SETTINGS
# ============================================================

POLAR_DIR = "S1210"
PROP_DIR = "UIUCSmall"

# ----------------------------
# Flight condition
# ----------------------------
RHO = 1.225
MU = 1.81e-5
G = 9.80665

V_INF = 3.0
INCIDENCE_DEG = 20.0
MASS_KG = 3.4
W_N = MASS_KG * G

# ----------------------------
# Geometry sweep
# ----------------------------
SPAN_GRID_M = np.array([2.0])                     # force 2 m study if desired
ROOT_CHORD_GRID_M = np.linspace(0.20, 0.3, 25)  # widened to help 2 m converge
TAPER_GRID = np.array([0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50])

# ----------------------------
# Propulsion sweep
# ----------------------------
NPROPS_GRID = np.arange(4, 80, 2)
RPM_SWEEP_RESOLUTION = 250
PROP_MASS_PER_UNIT_KG = 0.030      # lowered from 0.045 to make dense DP more realistic
PROP_SYSTEM_MASS_CAP_KG = 2.0 * 0.45359237

# ----------------------------
# Aerodynamic model
# ----------------------------
RE_CAP = 1.0e6
OSWALD_E = 0.80
TRIM_DRAG_FACTOR = 1.07
CD0_NOMINAL = 0.065

# These are profile-drag multipliers for local blown/unblown sections.
# Keep them simple for now; can refine later.
BLOWN_PROFILE_DRAG_FACTOR = 1.05
UNBLOWN_PROFILE_DRAG_FACTOR = 1.00

# ----------------------------
# Blown wing assumptions
# ----------------------------
K_SPAN_EXPANSION = 0.8            # slightly relaxed from earlier
SLIPSTREAM_RELAX = 0.5
MAX_BLOW_ITERS = 80
MAX_COUPLED_ITERS = 60
TOL_VEFF = 1e-4
TOL_THRUST = 1e-3

# actuator-disk style coupling from thrust -> effective blown velocity
# T = 2 rho A_disk vi^2, Veff = Vinf + 2 vi
USE_THRUST_TO_VEFF_COUPLING = True

# ----------------------------
# Electrical model
# ----------------------------
ETA_PROP = 0.75
ETA_MOTOR = 0.88
ETA_ESC = 0.98
ETA_TOTAL = ETA_PROP * ETA_MOTOR * ETA_ESC
AVIONICS_W = 10.0

# ----------------------------
# Ranking / reporting
# ----------------------------
THRUST_MARGIN = 1.10
TOP_K_TO_PRINT = 20


# ============================================================
# DATA CLASSES
# ============================================================

@dataclass
class Polar:
    Re: float
    alpha_deg: np.ndarray
    cl: np.ndarray
    cd: Optional[np.ndarray] = None

    @property
    def cl_max(self) -> float:
        return float(np.nanmax(self.cl))


@dataclass
class PropCurve:
    name: str
    diameter_in: float
    pitch_in: float
    diameter_m: float
    rpm: np.ndarray
    ct: np.ndarray
    cp: np.ndarray

    @property
    def rpm_min(self) -> float:
        return float(np.min(self.rpm))

    @property
    def rpm_max(self) -> float:
        return float(np.max(self.rpm))


@dataclass
class CandidateResult:
    span_m: float
    root_chord_m: float
    taper: float
    tip_chord_m: float
    area_m2: float
    ar: float
    mac_m: float
    nprops: int
    prop_name: str
    prop_diameter_in: float
    prop_pitch_in: float
    blown_eta: float
    veff_mps: float
    re_blow: float
    clmax_unblown: float
    clmax_blow: float
    cl_required_ref: float
    cl_required_blown_section: float
    cl_required_unblown_section: float
    drag_total_N: float
    drag_unblown_N: float
    drag_blown_N: float
    thrust_req_N: float
    thrust_avail_N: float
    rpm_operating: float
    shaft_power_W: float
    electric_power_W: float
    prop_system_mass_kg: float
    total_rank_metric: Tuple[float, float, int]


# ============================================================
# AIRFOIL POLAR PARSING
# ============================================================

def _parse_reynolds_from_header(lines: List[str]) -> Optional[float]:
    header = "\n".join(lines[:80])

    m = re.search(
        r"Re\s*=\s*([0-9]*\.?[0-9]+)\s*e\s*([+-]?\d+)",
        header,
        re.IGNORECASE
    )
    if m:
        base = float(m.group(1))
        exp = int(m.group(2))
        return base * (10.0 ** exp)

    m = re.search(
        r"Re\s*=\s*([0-9]*\.?[0-9]+(?:e[+-]?\d+)?)",
        header,
        re.IGNORECASE
    )
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            pass

    return None


def load_polar_txt(path: str) -> Polar:
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.read().splitlines()

    Re = _parse_reynolds_from_header(lines)

    if Re is None:
        base = os.path.basename(path)
        m = re.search(r"Re\s*([0-9]+(?:\.[0-9]+)?)", base, re.IGNORECASE)
        if not m:
            raise ValueError(f"Could not parse Reynolds number from file name: {base}")
        Re = float(m.group(1))
        if Re < 20.0:
            Re *= 1e6

    rows = []
    for ln in lines:
        s = ln.strip()
        if not s:
            continue

        s_low = s.lower()
        if (
            "alpha" in s_low
            or "xtrf" in s_low
            or "mach =" in s_low
            or "ncrit" in s_low
            or "calculated polar" in s_low
            or "reynolds number fixed" in s_low
            or set(s) <= {"-", " "}
        ):
            continue

        parts = s.split()
        if len(parts) < 2:
            continue

        try:
            alpha = float(parts[0])
            cl = float(parts[1])
            cd = float(parts[2]) if len(parts) >= 3 else np.nan
            rows.append((alpha, cl, cd))
        except ValueError:
            continue

    if len(rows) < 8:
        raise ValueError(f"Not enough numeric polar rows found in {path}")

    arr = np.array(rows, dtype=float)
    idx = np.argsort(arr[:, 0])
    cd_col = arr[idx, 2]
    cd_out = cd_col if np.isfinite(cd_col).any() else None

    return Polar(
        Re=float(Re),
        alpha_deg=arr[idx, 0],
        cl=arr[idx, 1],
        cd=cd_out
    )


class PolarDatabase:
    def __init__(self, polars: List[Polar]):
        if not polars:
            raise ValueError("No polars loaded.")
        self.polars = sorted(polars, key=lambda p: p.Re)
        self.re_list = np.array([p.Re for p in self.polars], dtype=float)
        self.clmax_list = np.array([p.cl_max for p in self.polars], dtype=float)

    def re_bounds(self) -> Tuple[float, float]:
        return float(np.min(self.re_list)), float(np.max(self.re_list))

    def clmax_at(self, Re: float) -> float:
        Re_c = float(np.clip(Re, np.min(self.re_list), np.max(self.re_list)))

        if Re_c <= self.re_list[0]:
            return float(self.clmax_list[0])
        if Re_c >= self.re_list[-1]:
            return float(self.clmax_list[-1])

        i_hi = int(np.searchsorted(self.re_list, Re_c, side="left"))
        r0, r1 = self.re_list[i_hi - 1], self.re_list[i_hi]
        c0, c1 = self.clmax_list[i_hi - 1], self.clmax_list[i_hi]

        x0, x1 = math.log10(r0), math.log10(r1)
        x = math.log10(Re_c)
        t = (x - x0) / (x1 - x0)
        return float((1.0 - t) * c0 + t * c1)


# ============================================================
# PROP PARSING
# ============================================================

def parse_prop_name(filename: str) -> Tuple[float, float]:
    base = os.path.splitext(os.path.basename(filename))[0]

    m = re.search(r"(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)", base, re.IGNORECASE)
    if m:
        d = float(m.group(1))
        p = float(m.group(2))
        if d > 20.0 and p > 20.0:
            return d / 25.4, p / 25.4
        return d, p

    m = re.search(r"(\d+(?:\.\d+)?)\s*in", base, re.IGNORECASE)
    if m:
        d = float(m.group(1))
        return d, 0.0

    raise ValueError(f"Could not parse prop size from file name: {os.path.basename(filename)}")


def load_prop_txt(path: str) -> PropCurve:
    diameter_in, pitch_in = parse_prop_name(path)

    rpm_vals = []
    ct_vals = []
    cp_vals = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            s = ln.strip()
            if not s:
                continue
            if any(k in s.lower() for k in ["rpm", "ct", "cp"]):
                continue

            parts = re.split(r"[\s,]+", s)
            if len(parts) < 3:
                continue

            try:
                rpm = float(parts[0])
                ct = float(parts[1])
                cp = float(parts[2])
            except ValueError:
                continue

            if rpm <= 0.0:
                continue

            rpm_vals.append(rpm)
            ct_vals.append(ct)
            cp_vals.append(cp)

    if len(rpm_vals) < 3:
        raise ValueError(f"Not enough prop data found in {path}")

    rpm = np.array(rpm_vals, dtype=float)
    ct = np.array(ct_vals, dtype=float)
    cp = np.array(cp_vals, dtype=float)

    idx = np.argsort(rpm)
    rpm = rpm[idx]
    ct = ct[idx]
    cp = cp[idx]

    return PropCurve(
        name=os.path.splitext(os.path.basename(path))[0],
        diameter_in=diameter_in,
        pitch_in=pitch_in,
        diameter_m=diameter_in * 0.0254,
        rpm=rpm,
        ct=ct,
        cp=cp
    )


class PropDatabase:
    def __init__(self, props: List[PropCurve]):
        if not props:
            raise ValueError("No prop files loaded.")
        self.props = props

    def interp_ct(self, prop: PropCurve, rpm: float) -> float:
        return float(np.interp(rpm, prop.rpm, prop.ct))

    def interp_cp(self, prop: PropCurve, rpm: float) -> float:
        return float(np.interp(rpm, prop.rpm, prop.cp))


# ============================================================
# GEOMETRY / AERO / PROPULSION
# ============================================================

def wing_area(span_m: float, root_chord_m: float, taper: float) -> float:
    tip = root_chord_m * taper
    return 0.5 * span_m * (root_chord_m + tip)

def tip_chord(root_chord_m: float, taper: float) -> float:
    return root_chord_m * taper

def mean_aerodynamic_chord(root_chord_m: float, taper: float) -> float:
    return (2.0 / 3.0) * root_chord_m * ((1 + taper + taper**2) / (1 + taper))

def aspect_ratio(span_m: float, area_m2: float) -> float:
    return span_m**2 / area_m2

def reynolds(V: float, c: float) -> float:
    return RHO * V * c / MU

def k_induced(ar: float, e: float) -> float:
    return 1.0 / (math.pi * e * ar)

def packing_feasible(span_m: float, nprops: int, D_m: float, kexp: float) -> bool:
    pitch = span_m / nprops
    return pitch >= kexp * D_m

def blown_fraction_eta(span_m: float, nprops: int, D_m: float, kexp: float) -> float:
    return min(1.0, (nprops * kexp * D_m) / span_m)

def propulsion_system_mass(nprops: int) -> float:
    return nprops * PROP_MASS_PER_UNIT_KG

def total_disk_area(nprops: int, D_m: float) -> float:
    return nprops * math.pi * (D_m / 2.0) ** 2

def thrust_from_prop(ct: float, rpm: float, D_m: float) -> float:
    n = rpm / 60.0
    return ct * RHO * (n**2) * (D_m**4)

def shaft_power_from_prop(cp: float, rpm: float, D_m: float) -> float:
    n = rpm / 60.0
    return cp * RHO * (n**3) * (D_m**5)

def veff_from_total_thrust(T_total: float, nprops: int, D_m: float) -> float:
    """
    Ideal actuator disk style coupling:
    T = 2 rho A vi^2
    Veff = Vinf + 2 vi
    """
    A = total_disk_area(nprops, D_m)
    if A <= 1e-12:
        return V_INF
    vi = math.sqrt(max(T_total, 0.0) / (2.0 * RHO * A))
    return V_INF + 2.0 * vi


# ============================================================
# TWO-ZONE LIFT / DRAG MODEL
# ============================================================

def two_zone_lift_drag(
    W_total_N: float,
    span_m: float,
    root_chord_m: float,
    taper: float,
    eta: float,
    veff: float,
    polars: PolarDatabase
) -> Optional[Dict[str, float]]:
    """
    Two-zone wing model:
    - unblown area = (1-eta) S at q_inf
    - blown area   = eta S at q_blow

    Assumption for first pass:
    - each zone runs up to its local CLmax when the solver is trying to close lift
    - drag is built from local profile + local induced drag, area-weighted by zone

    Returns a dict if feasible, else None.
    """
    S = wing_area(span_m, root_chord_m, taper)
    ar = aspect_ratio(span_m, S)
    c_mac = mean_aerodynamic_chord(root_chord_m, taper)

    S_u = (1.0 - eta) * S
    S_b = eta * S

    q_inf = 0.5 * RHO * V_INF**2
    q_b = 0.5 * RHO * veff**2

    Re_inf = reynolds(V_INF, c_mac)
    Re_b = reynolds(veff, c_mac)

    if Re_inf > RE_CAP or Re_b > RE_CAP:
        return None

    clmax_u = polars.clmax_at(Re_inf)
    clmax_b = polars.clmax_at(Re_b)

    # Let unblown section operate at local CLmax first
    L_u_max = q_inf * S_u * clmax_u
    L_remaining = W_total_N - L_u_max

    if L_remaining <= 0.0:
        cl_req_u = W_total_N / max(q_inf * S_u, 1e-12) if S_u > 0 else 0.0
        cl_req_b = 0.0
        L_u = W_total_N
        L_b = 0.0
    else:
        if S_b <= 1e-12 or q_b <= 1e-12:
            return None

        cl_req_u = clmax_u
        cl_req_b = L_remaining / (q_b * S_b)

        if cl_req_b > clmax_b:
            return None

        L_u = L_u_max
        L_b = L_remaining

    # freestream reference CL only for reporting
    cl_ref = W_total_N / max(q_inf * S, 1e-12)

    # zone drag
    k = k_induced(ar, OSWALD_E)

    # local CDs
    cd_u = TRIM_DRAG_FACTOR * (
        UNBLOWN_PROFILE_DRAG_FACTOR * CD0_NOMINAL
        + k * cl_req_u**2
    )
    cd_b = TRIM_DRAG_FACTOR * (
        BLOWN_PROFILE_DRAG_FACTOR * CD0_NOMINAL
        + k * cl_req_b**2
    )

    D_u = q_inf * S_u * cd_u
    D_b = q_b * S_b * cd_b
    D_total = D_u + D_b

    return {
        "S": S,
        "AR": ar,
        "MAC": c_mac,
        "q_inf": q_inf,
        "q_b": q_b,
        "Re_inf": Re_inf,
        "Re_b": Re_b,
        "clmax_u": clmax_u,
        "clmax_b": clmax_b,
        "cl_req_u": cl_req_u,
        "cl_req_b": cl_req_b,
        "cl_ref": cl_ref,
        "L_u": L_u,
        "L_b": L_b,
        "D_u": D_u,
        "D_b": D_b,
        "D_total": D_total,
    }


def solve_coupled_operating_point(
    W_total_N: float,
    span_m: float,
    root_chord_m: float,
    taper: float,
    nprops: int,
    prop: PropCurve,
    polars: PolarDatabase,
    pdb: PropDatabase
) -> Optional[Dict[str, float]]:
    """
    Coupled solve:
    rpm -> thrust -> Veff -> two-zone lift/drag -> required thrust
    iterate until thrust and drag close with margin
    """
    eta = blown_fraction_eta(span_m, nprops, prop.diameter_m, K_SPAN_EXPANSION)
    if eta <= 1e-8:
        return None

    # start near low-middle RPM
    rpm = max(prop.rpm_min, min(prop.rpm_max, 0.5 * (prop.rpm_min + prop.rpm_max)))

    for _ in range(MAX_COUPLED_ITERS):
        ct = pdb.interp_ct(prop, rpm)
        cp = pdb.interp_cp(prop, rpm)

        T_per = thrust_from_prop(ct, rpm, prop.diameter_m)
        P_per = shaft_power_from_prop(cp, rpm, prop.diameter_m)

        T_total = nprops * T_per
        P_shaft = nprops * P_per
        P_elec = P_shaft / max(ETA_TOTAL, 1e-9) + AVIONICS_W

        veff = veff_from_total_thrust(T_total, nprops, prop.diameter_m) if USE_THRUST_TO_VEFF_COUPLING else V_INF

        zone = two_zone_lift_drag(
            W_total_N=W_total_N,
            span_m=span_m,
            root_chord_m=root_chord_m,
            taper=taper,
            eta=eta,
            veff=veff,
            polars=polars
        )
        if zone is None:
            return None

        T_req = THRUST_MARGIN * zone["D_total"]

        # Need both lift feasibility and thrust closure
        err = T_total - T_req

        if abs(err) < TOL_THRUST:
            return {
                "rpm": rpm,
                "T_total": T_total,
                "P_shaft": P_shaft,
                "P_elec": P_elec,
                "veff": veff,
                "eta": eta,
                **zone,
            }

        # update rpm using thrust ratio
        if T_total <= 1e-9:
            return None

        rpm_new = rpm * math.sqrt(max(T_req, 1e-9) / T_total)
        rpm_new = float(np.clip(rpm_new, prop.rpm_min, prop.rpm_max))

        if abs(rpm_new - rpm) < RPM_SWEEP_RESOLUTION * 0.25:
            # if close enough in RPM, do one final check at clipped point
            rpm = rpm_new
            ct = pdb.interp_ct(prop, rpm)
            cp = pdb.interp_cp(prop, rpm)
            T_per = thrust_from_prop(ct, rpm, prop.diameter_m)
            P_per = shaft_power_from_prop(cp, rpm, prop.diameter_m)
            T_total = nprops * T_per
            P_shaft = nprops * P_per
            P_elec = P_shaft / max(ETA_TOTAL, 1e-9) + AVIONICS_W
            veff = veff_from_total_thrust(T_total, nprops, prop.diameter_m) if USE_THRUST_TO_VEFF_COUPLING else V_INF
            zone = two_zone_lift_drag(
                W_total_N=W_total_N,
                span_m=span_m,
                root_chord_m=root_chord_m,
                taper=taper,
                eta=eta,
                veff=veff,
                polars=polars
            )
            if zone is None:
                return None
            T_req = THRUST_MARGIN * zone["D_total"]
            if T_total >= T_req:
                return {
                    "rpm": rpm,
                    "T_total": T_total,
                    "P_shaft": P_shaft,
                    "P_elec": P_elec,
                    "veff": veff,
                    "eta": eta,
                    **zone,
                }
            return None

        rpm = (1.0 - SLIPSTREAM_RELAX) * rpm + SLIPSTREAM_RELAX * rpm_new

    return None


# ============================================================
# MAIN OPTIMIZER
# ============================================================

def run_optimizer():
    polar_paths = sorted(glob.glob(os.path.join(POLAR_DIR, "*.txt")))
    if not polar_paths:
        raise FileNotFoundError(f"No polar files found in {POLAR_DIR}")

    polars = []
    for p in polar_paths:
        try:
            polars.append(load_polar_txt(p))
        except Exception as e:
            print(f"Skipping polar {os.path.basename(p)}: {e}")

    if not polars:
        raise RuntimeError("No valid S1210 polars loaded.")

    polar_db = PolarDatabase(polars)
    re_min, re_max = polar_db.re_bounds()

    prop_paths = sorted(glob.glob(os.path.join(PROP_DIR, "*.txt")))
    if not prop_paths:
        raise FileNotFoundError(f"No prop files found in {PROP_DIR}")

    prop_curves = []
    for p in prop_paths:
        try:
            prop_curves.append(load_prop_txt(p))
        except Exception as e:
            print(f"Skipping prop file {os.path.basename(p)}: {e}")

    if not prop_curves:
        raise RuntimeError("No valid prop files loaded.")

    prop_db = PropDatabase(prop_curves)

    print("\n=== Loaded airfoil polars ===")
    print(f"Count: {len(polars)}")
    print(f"Re range: {re_min:.0f} to {re_max:.0f}")

    print("\n=== Loaded prop curves ===")
    for p in prop_curves:
        print(
            f"{p.name}: D={p.diameter_in:.2f} in, pitch={p.pitch_in:.2f} in, "
            f"RPM={p.rpm_min:.0f}-{p.rpm_max:.0f}"
        )

    feasible: List[CandidateResult] = []
    fail_counts: Dict[str, int] = {
        "mass_cap": 0,
        "packing": 0,
        "coupled_no_solution": 0,
    }

    for span_m in SPAN_GRID_M:
        for cr in ROOT_CHORD_GRID_M:
            for taper in TAPER_GRID:
                ct = tip_chord(cr, taper)
                S = wing_area(span_m, cr, taper)
                ar = aspect_ratio(span_m, S)
                mac = mean_aerodynamic_chord(cr, taper)

                for nprops in NPROPS_GRID:
                    m_prop = propulsion_system_mass(nprops)
                    if m_prop > PROP_SYSTEM_MASS_CAP_KG:
                        fail_counts["mass_cap"] += 1
                        continue

                    W_total = W_N + m_prop * G

                    for prop in prop_curves:
                        if not packing_feasible(span_m, int(nprops), prop.diameter_m, K_SPAN_EXPANSION):
                            fail_counts["packing"] += 1
                            continue

                        coupled = solve_coupled_operating_point(
                            W_total_N=W_total,
                            span_m=span_m,
                            root_chord_m=cr,
                            taper=taper,
                            nprops=int(nprops),
                            prop=prop,
                            polars=polar_db,
                            pdb=prop_db
                        )

                        if coupled is None:
                            fail_counts["coupled_no_solution"] += 1
                            continue

                        feasible.append(
                            CandidateResult(
                                span_m=span_m,
                                root_chord_m=cr,
                                taper=taper,
                                tip_chord_m=ct,
                                area_m2=S,
                                ar=ar,
                                mac_m=mac,
                                nprops=int(nprops),
                                prop_name=prop.name,
                                prop_diameter_in=prop.diameter_in,
                                prop_pitch_in=prop.pitch_in,
                                blown_eta=coupled["eta"],
                                veff_mps=coupled["veff"],
                                re_blow=coupled["Re_b"],
                                clmax_unblown=coupled["clmax_u"],
                                clmax_blow=coupled["clmax_b"],
                                cl_required_ref=coupled["cl_ref"],
                                cl_required_blown_section=coupled["cl_req_b"],
                                cl_required_unblown_section=coupled["cl_req_u"],
                                drag_total_N=coupled["D_total"],
                                drag_unblown_N=coupled["D_u"],
                                drag_blown_N=coupled["D_b"],
                                thrust_req_N=THRUST_MARGIN * coupled["D_total"],
                                thrust_avail_N=coupled["T_total"],
                                rpm_operating=coupled["rpm"],
                                shaft_power_W=coupled["P_shaft"],
                                electric_power_W=coupled["P_elec"],
                                prop_system_mass_kg=m_prop,
                                total_rank_metric=(coupled["P_elec"], m_prop, int(nprops))
                            )
                        )

    print("\n=== FAILURE COUNTS ===")
    for k, v in fail_counts.items():
        print(f"{k}: {v}")

    if not feasible:
        print("\nNo feasible designs found.")
        print("What I should check next:")
        print("- increase chord range more")
        print("- reduce mass or prop mass per unit")
        print("- relax packing assumptions further")
        print("- allow more props or smaller props")
        print("- revisit CD0 / blown drag factors")
        return

    feasible_sorted = sorted(feasible, key=lambda x: x.total_rank_metric)

    print("\n=== TOP FEASIBLE DESIGNS ===")
    for i, r in enumerate(feasible_sorted[:TOP_K_TO_PRINT], start=1):
        print(
            f"{i:2d}) "
            f"Prop={r.prop_name:30s} | N={r.nprops:2d} | RPM={r.rpm_operating:6.0f} | "
            f"b={r.span_m:.2f} m | cr={r.root_chord_m:.3f} m | ct={r.tip_chord_m:.3f} m | taper={r.taper:.2f} | "
            f"S={r.area_m2:.3f} m^2 | AR={r.ar:.2f} | eta={r.blown_eta:.3f} | "
            f"Veff={r.veff_mps:.2f} m/s | Re_blow={r.re_blow:.0f} | "
            f"CLref={r.cl_required_ref:.2f} | CLu_req={r.cl_required_unblown_section:.2f} | "
            f"CLb_req={r.cl_required_blown_section:.2f} | CLmax_u={r.clmax_unblown:.2f} | CLmax_b={r.clmax_blow:.2f} | "
            f"Dtot={r.drag_total_N:.2f} N (Du={r.drag_unblown_N:.2f}, Db={r.drag_blown_N:.2f}) | "
            f"Treq={r.thrust_req_N:.2f} N | Tavail={r.thrust_avail_N:.2f} N | "
            f"Pshaft={r.shaft_power_W:.1f} W | Pelec={r.electric_power_W:.1f} W | "
            f"m_prop={r.prop_system_mass_kg:.3f} kg"
        )

    best = feasible_sorted[0]

    print("\n=== BEST FIRST-PASS CONCEPT ===")
    print(f"Prop:                 {best.prop_name}")
    print(f"Motor count:          {best.nprops}")
    print(f"Operating RPM:        {best.rpm_operating:.0f}")
    print(f"Span:                 {best.span_m:.3f} m")
    print(f"Root chord:           {best.root_chord_m:.3f} m")
    print(f"Tip chord:            {best.tip_chord_m:.3f} m")
    print(f"Taper ratio:          {best.taper:.3f}")
    print(f"Wing area:            {best.area_m2:.3f} m^2")
    print(f"Aspect ratio:         {best.ar:.3f}")
    print(f"Blown span fraction:  {best.blown_eta:.3f}")
    print(f"Required Veff:        {best.veff_mps:.3f} m/s")
    print(f"Re_blow:              {best.re_blow:.0f}")
    print(f"CL ref (freestream):  {best.cl_required_ref:.3f}")
    print(f"CL req unblown:       {best.cl_required_unblown_section:.3f}")
    print(f"CL req blown:         {best.cl_required_blown_section:.3f}")
    print(f"CLmax unblown:        {best.clmax_unblown:.3f}")
    print(f"CLmax blown:          {best.clmax_blow:.3f}")
    print(f"Total drag:           {best.drag_total_N:.3f} N")
    print(f"Unblown drag:         {best.drag_unblown_N:.3f} N")
    print(f"Blown drag:           {best.drag_blown_N:.3f} N")
    print(f"Thrust required:      {best.thrust_req_N:.3f} N")
    print(f"Thrust available:     {best.thrust_avail_N:.3f} N")
    print(f"Shaft power:          {best.shaft_power_W:.2f} W")
    print(f"Electrical power:     {best.electric_power_W:.2f} W")
    print(f"Propulsion mass:      {best.prop_system_mass_kg:.3f} kg")

    summary = {
        "best_prop": best.prop_name,
        "best_motor_count": int(best.nprops),
        "best_rpm": float(best.rpm_operating),
        "best_span_m": float(best.span_m),
        "best_root_chord_m": float(best.root_chord_m),
        "best_tip_chord_m": float(best.tip_chord_m),
        "best_taper": float(best.taper),
        "best_area_m2": float(best.area_m2),
        "best_AR": float(best.ar),
        "best_power_W": float(best.electric_power_W),
        "best_cl_ref": float(best.cl_required_ref),
        "best_cl_u_req": float(best.cl_required_unblown_section),
        "best_cl_b_req": float(best.cl_required_blown_section),
        "best_clmax_u": float(best.clmax_unblown),
        "best_clmax_b": float(best.clmax_blow),
    }

    print("\n=== SUMMARY DICT ===")
    print(summary)


if __name__ == "__main__":
    run_optimizer()