#!/usr/bin/env python3
"""Seekfree CYT4BB Flight Data Analyzer.
Usage: python driver.py [path/to/test.txt]
Parses the printf format, identifies handheld segment, analyzes all control loops.
"""

import sys, os, re, math
from pathlib import Path

# ── 1. Parse ──────────────────────────────────
FIELD_NAMES = [
    "FA","valid","fresh","H","Vz",
    "rx","ry","tvx","tvy","vx","vy","fx","fy","imx","imy",
    "fw","pc","rc","evx","evy","op","or","ivx","ivy",
    "ROL","PIT","TGTY","YR","YRC","YL","YI",
    "cv","cx","cy","u","v","ar","mg","rx_cnt",
    "PO","RO","YO","PR","RR","PD","RD",
]

LINE_PATTERN = re.compile(
    r'(?:UPF\s)?FA:(-?\d+)\s+valid:(-?\d+)\s+fresh:(-?\d+)\s+H:([-\d.]+)\s+Vz:([-\d.]+)\s+'
    r'rx:([-\d.]+)\s+ry:([-\d.]+)\s+tvx:([-\d.]+)\s+tvy:([-\d.]+)\s+'
    r'vx:([-\d.]+)\s+vy:([-\d.]+)\s+fx:([-\d.]+)\s+fy:([-\d.]+)\s+imx:([-\d.]+)\s+imy:([-\d.]+)\s+'
    r'fw:([-\d.]+)\s+pc:([-\d.]+)\s+rc:([-\d.]+)\s+evx:([-\d.]+)\s+evy:([-\d.]+)\s+'
    r'op:([-\d.]+)\s+or:([-\d.]+)\s+ivx:([-\d.]+)\s+ivy:([-\d.]+)\s+'
    r'ROL:([-\d.]+)\s+PIT:([-\d.]+)\s+TGTY:([-\d.]+)\s+YR:([-\d.]+)\s+YRC:([-\d.]+)\s+'
    r'YL:([-\d.]+)\s+YI:([-\d.]+)\s+cv:(-?\d+)\s+cx:([-\d.]+)\s+cy:([-\d.]+)\s+'
    r'u:(-?\d+)\s+v:(-?\d+)\s+ar:(-?\d+)\s+mg:(-?\d+)\s+rx:(\d+)\s+'
    r'PO:([-\d.]+)\s+RO:([-\d.]+)\s+YO:([-\d.]+)\s+PR:([-\d.]+)\s+RR:([-\d.]+)'
    r'(?:\s+PD:([-\d.]+)\s+RD:([-\d.]+))?'
)

INT_FIELDS = {1,2,3,32,34,35,36,37,38}

def parse_line(line):
    line = line.strip()
    if not line or line.startswith("==") or line.startswith("Boot"):
        return None
    m = LINE_PATTERN.match(line)
    if not m:
        return None
    vals = []
    for i in range(1, 47):
        g = m.group(i)
        if g is None:
            vals.append(0.0)
        elif i in INT_FIELDS:
            try:
                vals.append(int(g))
            except:
                vals.append(0)
        else:
            try:
                vals.append(float(g))
            except:
                vals.append(0.0)
    if len(vals) < 46:
        vals.extend([0.0, 0.0])
    return dict(zip(FIELD_NAMES, vals))

def read_data(path):
    with open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    data = [parse_line(l) for l in lines if parse_line(l)]
    return data

# ── 2. Handheld detection ──────────────────────
def find_handheld(data):
    n = len(data)
    if n < 20:
        return None
    h_start = None
    for i in range(n-1, max(0, n-100), -1):
        d = data[i]
        if abs(d["Vz"]) > 40 or d["FA"] == 0:
            h_start = i
        elif h_start is not None:
            break
    if h_start is None:
        h_start = max(0, n - 15)
    return (h_start, n-1)

# ── 3. Analysis functions ─────────────────────
def analyze_optical_flow(flight):
    fresh_r = sum(1 for d in flight if d["fresh"]) / max(len(flight),1)
    valid_r = sum(1 for d in flight if d["valid"]) / max(len(flight),1)
    vxa = [abs(d["vx"]) for d in flight]
    max_vx = max(vxa) if vxa else 0
    avg_vx = sum(vxa)/len(vxa) if vxa else 0
    issues = []
    if fresh_r < 0.5: issues.append("frame_stale")
    if valid_r < 0.7: issues.append("low_validity")
    if max_vx > 60: issues.append(f"speed_spike_{max_vx:.0f}cm/s")
    st = "FAIL" if max_vx > 80 else "WARNING" if (max_vx > 60 or fresh_r < 0.7) else "OK"
    return {"status":st, "issues":issues}

def analyze_camera(flight):
    cv_r = sum(1 for d in flight if d["cv"]) / max(len(flight),1)
    mg_mx = max(d["mg"] for d in flight)
    ar_mx = max(d["ar"] for d in flight)
    issues = []
    if cv_r < 0.01: issues.append(f"cv=1_never_(mg={mg_mx},ar={ar_mx})")
    elif cv_r < 0.8: issues.append(f"cam_dropout_{cv_r:.0%}")
    if mg_mx < 30: issues.append(f"low_mg_{mg_mx}")
    if ar_mx < 4: issues.append(f"small_area_{ar_mx}")
    st = "FAIL" if cv_r < 0.01 else "WARNING" if cv_r < 0.8 else "OK"
    return {"status":st, "issues":issues}

def analyze_speed_loop(flight):
    sat_p = sum(1 for d in flight if abs(d["pc"]) >= 11.9)/max(len(flight),1)
    sat_r = sum(1 for d in flight if abs(d["rc"]) >= 11.9)/max(len(flight),1)
    ev = [abs(d["evx"]) for d in flight]
    avg_ev = sum(ev)/len(ev) if ev else 0
    max_ev = max(ev) if ev else 0
    issues = []
    if sat_p > 0.3: issues.append(f"pc_sat_{sat_p:.0%}")
    if sat_r > 0.3: issues.append(f"rc_sat_{sat_r:.0%}")
    if avg_ev > 15: issues.append(f"ev_mean_{avg_ev:.1f}cm/s")
    st = "FAIL" if (sat_p>0.5 or sat_r>0.5 or avg_ev>20) else "WARNING" if (sat_p>0.2 or sat_r>0.2) else "OK"
    return {"status":st, "issues":issues}

def analyze_position_loop(flight):
    engaged = [d for d in flight if d["cv"] and d["H"] >= 40 and abs(d["Vz"]) < 12]
    ratio = len(engaged)/max(len(flight),1)
    issues = []
    if not engaged: issues.append("posloop_never_engaged (cv=0 or H<40 or Vz>12)")
    elif ratio < 0.2: issues.append(f"posloop_intermittent_{ratio:.0%}")
    st = "FAIL" if not engaged else "WARNING" if ratio < 0.5 else "OK"
    return {"status":st, "issues":issues}

def analyze_attitude_pid(flight):
    po_ro_sat = sum(1 for d in flight if abs(d["PO"])>=1190 or abs(d["RO"])>=1190)/max(len(flight),1)
    pr_sat = sum(1 for d in flight if abs(d["PR"])>=59)/max(len(flight),1)
    rr_sat = sum(1 for d in flight if abs(d["RR"])>=59)/max(len(flight),1)
    pd = [abs(d["PD"]) for d in flight if abs(d["PD"])<1e4]
    rd = [abs(d["RD"]) for d in flight if abs(d["RD"])<1e4]
    avg_pd = sum(pd)/len(pd) if pd else 0
    avg_rd = sum(rd)/len(rd) if rd else 0
    issues = []
    if avg_pd > 50: issues.append(f"PD_noisy_{avg_pd:.0f}")
    if avg_rd > 50: issues.append(f"RD_noisy_{avg_rd:.0f}")
    if po_ro_sat > 0.3: issues.append(f"mixer_sat_{po_ro_sat:.0%}")
    if pr_sat > 0.3: issues.append(f"pitch_rate_sat_{pr_sat:.0%}")
    if rr_sat > 0.3: issues.append(f"roll_rate_sat_{rr_sat:.0%}")
    st = "FAIL" if po_ro_sat > 0.5 else "WARNING" if (avg_pd>50 or avg_rd>50 or pr_sat>0.5) else "OK"
    return {"status":st, "issues":issues}

def analyze_yaw(flight):
    yi = [d["YI"] for d in flight]
    yi_drift = abs(yi[-1] - yi[0]) if len(yi) > 10 else 0
    yi_max = max(yi) if yi else 0
    yrc_max = max(abs(d["YRC"]) for d in flight)
    issues = []
    if yi_drift > 30: issues.append(f"YI_drift_{yi_drift:.0f}")
    if yi_max > 50: issues.append(f"YI_max_{yi_max:.0f}")
    if yrc_max > 30: issues.append(f"YRC_max_{yrc_max:.0f}dps")
    st = "FAIL" if (yi_drift > 50 or yi_max > 100) else "WARNING" if (yi_drift > 20 or yi_max > 50) else "OK"
    return {"status":st, "issues":issues}

def analyze_climb(flight):
    vz = [abs(d["Vz"]) for d in flight]
    vz_max = max(vz) if vz else 0
    vz_avg = sum(vz)/len(vz) if vz else 0
    h_rng = max(d["H"] for d in flight) - min(d["H"] for d in flight)
    issues = []
    if vz_max > 50: issues.append(f"Vz_spike_{vz_max:.0f}cm/s")
    if vz_avg > 15: issues.append(f"Vz_mean_{vz_avg:.1f}cm/s")
    if h_rng > 30: issues.append(f"H_swing_{h_rng:.0f}cm")
    st = "FAIL" if (vz_max > 60 or h_rng > 50) else "WARNING" if (vz_max > 30 or h_rng > 20) else "OK"
    return {"status":st, "issues":issues}

# ── 4. Root cause + fix suggestions ────────────
def suggest_fixes(cam, spd, pos, att, yaw_r, climb, of):
    failures = []
    for name, mod in [("cam",cam),("spd",spd),("pos",pos),("att",att),("yaw",yaw_r),("climb",climb),("of",of)]:
        if mod.get("status") == "FAIL":
            failures.append(name)

    cm = cam.get("issues", [])
    mg_mx = max(len(cam.get("issues", [])), 0)

    if "cam" in failures and any("mg" in i or "area" in i for i in cm):
        cause = "Camera never locks target (mg too low or area too small)"
        fixes = [
            "A. Lower LED_THRESHOLD from 30 to 20",
            "B. Lower LED_MIN_AREA from 4 to 2",
            "C. Check LED power and camera angle",
            "D. Expand ROI_HALF from 30 to 40",
        ]
    elif "att" in failures:
        cause = "Attitude loop oscillation (D term amplifies gyro noise, or P saturates)"
        fixes = [
            "A. Set ATTITUDE_STYLE_MAPLEPID=0 (back to classic P-only)",
            "B. Lower KD from 1.50 to 0.05 and reduce d_lpf_alpha",
            "C. Increase FLOW_MAX_ANGLE_DEG from 12 to 15 for more headroom",
        ]
    elif "yaw" in failures:
        cause = "Yaw I-term windup (sustained bias not reset)"
        fixes = [
            "A. Lower pid_yaw_rate.ki from 0.10 to 0.05",
            "B. Lower pid_yaw_rate.i_limit from 200 to 100",
            "C. Verify yaw fault detection resets properly",
        ]
    elif "climb" in failures:
        cause = "Altitude oscillation (Vz spikes, altitude can't hold)"
        fixes = [
            "A. Lower CLIMB_UP_MAX_SPEED from 12 to 8",
            "B. Raise pid_alt_vel.kp from 7.5 to 10.0",
            "C. Lower pid_alt_pos.kp from 0.6 to 0.4",
        ]
    elif "speed_loop" in failures or "position_loop" in failures:
        cause = "Speed/position loop saturation (commands exceed actuator limits)"
        fixes = [
            "A. Increase FLOW_MAX_ANGLE_DEG from 12 to 15",
            "B. Lower CAM_POS_KP from 2.0 to 1.0",
            "C. Lower V_FOLLOW_MAX from 18 to 12",
        ]
    else:
        cause = "Multiple subsystems marginal; system diverges from combined effects"
        fixes = [
            "A. Fix camera detection first (cv never=1)",
            "B. Lower V_FOLLOW_MAX from 18 to 12",
            "C. Switch ATTITUDE_STYLE_MAPLEPID=0",
        ]
    return cause, fixes

# ── 5. Main analysis ──────────────────────────
def analyze(data):
    if not data:
        return {"error": "No valid data"}
    handheld = find_handheld(data)
    if handheld:
        flight = data[:handheld[0]]
    else:
        flight = data

    if len(flight) < 5:
        return {"error": f"Only {len(flight)} flight frames"}

    of = analyze_optical_flow(flight)
    cam = analyze_camera(flight)
    spd = analyze_speed_loop(flight)
    pos = analyze_position_loop(flight)
    att = analyze_attitude_pid(flight)
    yaw_r = analyze_yaw(flight)
    climb = analyze_climb(flight)

    cause, fixes = suggest_fixes(cam, spd, pos, att, yaw_r, climb, of)

    return {
        "total_frames": len(data),
        "flight_frames": len(flight),
        "handheld_frames": len(data) - len(flight) if handheld else 0,
        "optical_flow": of, "camera": cam, "speed_loop": spd,
        "position_loop": pos, "attitude_pid": att, "yaw": yaw_r, "climb": climb,
        "root_cause": cause, "fix_suggestions": fixes,
    }

def print_report(r):
    print("=" * 70)
    print("  FLIGHT DATA ANALYSIS REPORT")
    print("=" * 70)
    if "error" in r:
        print(f"  ERROR: {r['error']}"); return
    print(f"  Frames: total={r['total_frames']}  flight={r['flight_frames']}  handheld={r['handheld_frames']}")
    print()
    subs = [("OptFlow","optical_flow"),("Camera","camera"),("SpdLoop","speed_loop"),
            ("PosLoop","position_loop"),("AttPID","attitude_pid"),("YawPID","yaw"),("Climb","climb")]
    m = {"OK":" [PASS]","WARNING":" [WARN]","FAIL":" [FAIL]"}
    for label, key in subs:
        mod = r.get(key, {})
        s = mod.get("status", "?")
        iss = mod.get("issues", [])
        det = "; ".join(iss[:2]) if iss else "(ok)"
        print(f"  {m.get(s,' [?]')} [{label}] {det}")
    print()
    print("-" * 70)
    print(f"  Root cause: {r.get('root_cause','N/A')}")
    print()
    print("  Suggested fixes (ask me before applying):")
    for s in r.get("fix_suggestions", []):
        print(f"    {s}")
    print()
    print("  NOTE: I will NOT modify code without your confirmation.")
    print("=" * 70)

# ── 6. Entry ─────────────────────────────────
if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "test.txt"
    p = Path(target)
    if not p.is_absolute():
        for c in [Path.cwd()/target, Path(__file__).parents[2]/target]:
            if c.exists(): p = c; break
    print(f"Reading: {p.resolve()}")
    data = read_data(str(p.resolve()))
    if not data:
        print("No valid data lines."); sys.exit(1)
    r = analyze(data)
    print_report(r)
