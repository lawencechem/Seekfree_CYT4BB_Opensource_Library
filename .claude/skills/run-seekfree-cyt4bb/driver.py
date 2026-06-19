#!/usr/bin/env python3
"""
Seekfree CYT4BB Flight Controller Firmware Driver.

Build, flash, and monitor the quadcopter firmware for the Infineon
Traveo T2G CYT4BB7CEE dual-core MCU (CM7_0 flight + CM7_1 vision).

Usage:
  python driver.py build          Build both cores
  python driver.py build --core 7_0   Build only CM7_0
  python driver.py build --core 7_1   Build only CM7_1
  python driver.py flash          Flash via IAR C-SPY (hardware required)
  python driver.py verify         Check build outputs exist
  python driver.py info           Show build info and firmware details
  python driver.py clean          Clean build artifacts
"""

import argparse
import os
import shutil
import struct
import subprocess
import sys
from pathlib import Path

# Force UTF-8 output for Windows GBK terminals
os.environ.setdefault("PYTHONIOENCODING", "utf-8")

# driver.py is at .claude/skills/run-seekfree-cyt4bb/driver.py
# repo root is 3 levels up from the skill dir
REPO_ROOT = Path(__file__).resolve().parents[3]

# --- Paths (relative to REPO_ROOT) ---
IAR_ARM_BIN = Path("C:/Program Files/IAR Systems/Embedded Workbench 9.2/arm/bin")
IAR_COMMON_BIN = Path("C:/Program Files/IAR Systems/Embedded Workbench 9.2/common/bin")
IARBUILD = IAR_COMMON_BIN / "iarbuild.exe"
CSPYBAT = IAR_COMMON_BIN / "CSpyBat.exe"

EWP_7_0 = REPO_ROOT / "project/iar/project_config/cyt4bb7_cm_7_0.ewp"
EWP_7_1 = REPO_ROOT / "project/iar/project_config/cyt4bb7_cm_7_1.ewp"
EWD_7_0 = REPO_ROOT / "project/iar/project_config/cyt4bb7_cm_7_0.ewd"
EWD_7_1 = REPO_ROOT / "project/iar/project_config/cyt4bb7_cm_7_1.ewd"
WSDT = REPO_ROOT / "project/iar/settings/cyt4bb7.wsdt"

OUT_7_0 = REPO_ROOT / "project/iar/project_config/Debug_m7_0/Exe/cyt4bb7_cm_7_0.out"
HEX_7_0 = REPO_ROOT / "project/iar/project_config/Debug_m7_0/Exe/cyt4bb7_cm_7_0.hex"
OUT_7_1 = REPO_ROOT / "project/iar/project_config/Debug_m7_1/Exe/cyt4bb7_cm_7_1.out"
HEX_7_1 = REPO_ROOT / "project/iar/project_config/Debug_m7_1/Exe/cyt4bb7_cm_7_1.hex"

BUILD_DIRS = [
    REPO_ROOT / "project/iar/Debug_m7_0",
    REPO_ROOT / "project/iar/Debug_m7_1",
    REPO_ROOT / "project/iar/project_config/Debug_m7_0",
    REPO_ROOT / "project/iar/project_config/Debug_m7_1",
]


def require_iarbuild() -> Path:
    if not IARBUILD.exists():
        print(f"ERROR: IARBuild not found at {IARBUILD}", file=sys.stderr)
        print("IAR Embedded Workbench 9.2 is required for building.", file=sys.stderr)
        sys.exit(1)
    return IARBUILD


def check_ewp(path: Path, label: str):
    if not path.exists():
        print(f"ERROR: {label} project not found at {path}", file=sys.stderr)
        sys.exit(1)


def build_core(ewp: Path, label: str):
    """Build one core with IARBuild."""
    require_iarbuild()
    check_ewp(ewp, label)

    print(f"\n=== Building {label} ({ewp.name}) ===")
    result = subprocess.run(
        [str(IARBUILD), str(ewp), "-build", "Debug"],
        capture_output=False,
    )
    print(f"=== {label} exit code: {result.returncode} ===\n")
    return result.returncode


def cmd_build(args):
    """Build firmware for one or both cores."""
    cores_to_build = []
    if args.core:
        cores_to_build.append(args.core)
    else:
        cores_to_build = ["7_0", "7_1"]

    codes = []
    for core in cores_to_build:
        if core == "7_0":
            c = build_core(EWP_7_0, "CM7_0 (flight controller)")
        elif core == "7_1":
            c = build_core(EWP_7_1, "CM7_1 (vision processor)")
        else:
            print(f"Unknown core: {core}. Use 7_0 or 7_1.", file=sys.stderr)
            codes.append(1)
            continue
        codes.append(c)

    return 0 if all(c == 0 for c in codes) else 1


def cmd_flash(args):
    """Flash firmware via IAR C-SPY (requires hardware debug probe)."""
    require_iarbuild()

    if not CSPYBAT.exists():
        print(f"ERROR: CSpyBat not found at {CSPYBAT}", file=sys.stderr)
        return 1

    print("=== Flashing CM7_0 + CM7_1 via IAR C-SPY ===")
    print("NOTE: Requires a connected debug probe (J-Link/I-jet) and target hardware.\n")

    if args.core and args.core == "7_1":
        cores = ["7_1"]
    else:
        cores = ["7_0", "7_1"]

    for core in cores:
        if core == "7_0":
            ewd = EWD_7_0
            out = OUT_7_0
        else:
            ewd = EWD_7_1
            out = OUT_7_1

        if not ewd.exists():
            print(f"WARNING: Debugger settings not found: {ewd}")
            print(f"  Skipping flash for CM7_{core}.")
            continue
        if not out.exists():
            print(f"ERROR: Build output not found: {out}")
            print(f"  Run 'python driver.py build' first.")
            return 1

        print(f"  Flashing CM7_{core}...")
        result = subprocess.run(
            [str(CSPYBAT), str(ewd), str(out), "--flash_loader"],
            capture_output=False,
        )
        if result.returncode != 0:
            print(f"  Flash failed for CM7_{core} (exit {result.returncode})", file=sys.stderr)
            return result.returncode
        print(f"  CM7_{core} flashed successfully.")

    print("\n=== Flash complete ===")
    return 0


def cmd_verify(args):
    """Verify build outputs exist and show file info."""
    print("=== Build Output Verification ===\n")

    artifacts = [
        ("CM7_0 .out", OUT_7_0),
        ("CM7_0 .hex", HEX_7_0),
        ("CM7_1 .out", OUT_7_1),
        ("CM7_1 .hex", HEX_7_1),
    ]

    all_ok = True
    for label, path in artifacts:
        if path.exists():
            size_kb = path.stat().st_size / 1024
            print(f"  [OK] {label}: {size_kb:.1f} KiB  ({path.name})")
        else:
            print(f"  [MISS] {label}: NOT FOUND  ({path})")
            all_ok = False

    print()
    if all_ok:
        print("All build outputs present.")
        return 0
    else:
        print("Run 'python driver.py build' to build missing outputs.", file=sys.stderr)
        return 1


def print_hex_info(path: Path, label: str):
    """Print firmware file info."""
    if not path.exists():
        return
    size_kb = path.stat().st_size / 1024
    print(f"  {label}: {size_kb:.1f} KiB ({path.name})")


def cmd_info(args):
    """Show project and build information."""
    print("=== Seekfree CYT4BB Flight Controller Firmware ===\n")

    print("Target MCU: Infineon Traveo T2G CYT4BB7CEE")
    print("            Dual-core ARM Cortex-M7 @ 250 MHz")
    print("Cores:")
    print("  CM7_0: Flight controller (IMU, PID, mixing, PWM)")
    print("  CM7_1: Vision processor (MT9V03X camera, IPC)")
    print()

    print("Toolchain:")
    if IARBUILD.exists():
        print(f"  [OK] IAR Embedded Workbench 9.2")
        print(f"     {IARBUILD}")
    else:
        print(f"  [MISS] IAR Embedded Workbench not found at {IARBUILD}")
    print()

    print("Build outputs:")
    print_hex_info(HEX_7_0, "CM7_0 (flight)")
    print_hex_info(HEX_7_1, "CM7_1 (vision)")
    print()

    # Check IAR workspace
    ws = REPO_ROOT / "project/iar/cyt4bb7.eww"
    if ws.exists():
        print("IAR Workspace: project/iar/cyt4bb7.eww")

    # Check linker script
    icf = REPO_ROOT / "project/iar/icf/linker_directives_tviibh.icf"
    if icf.exists():
        icf_size = icf.stat().st_size
        print(f"Linker Script: project/iar/icf/linker_directives_tviibh.icf ({icf_size} bytes)")

    return 0


def cmd_clean(args):
    """Remove build artifacts."""
    print("=== Cleaning Build Artifacts ===\n")

    removed = 0
    for d in BUILD_DIRS:
        if d.exists():
            print(f"  Removing {d.relative_to(REPO_ROOT)}/")
            shutil.rmtree(d)
            removed += 1

    # Also clean IAR workspace settings cache
    settings_cache = REPO_ROOT / "project/iar/settings"
    for f in settings_cache.glob("*.wsdt"):
        print(f"  Keeping {f.relative_to(REPO_ROOT)} (workspace settings)")

    if removed == 0:
        print("  Nothing to clean.")
    else:
        print(f"\n  Removed {removed} build directory/directories.")
        print("  Run 'python driver.py build' to rebuild.")

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Seekfree CYT4BB Flight Controller Firmware Driver"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # build
    p_build = subparsers.add_parser("build", help="Build firmware (one or both cores)")
    p_build.add_argument("--core", choices=["7_0", "7_1"], help="Build only specified core")

    # flash
    p_flash = subparsers.add_parser("flash", help="Flash firmware via IAR C-SPY")
    p_flash.add_argument("--core", choices=["7_0", "7_1"], help="Flash only specified core")

    # verify
    subparsers.add_parser("verify", help="Verify build outputs")

    # info
    subparsers.add_parser("info", help="Show project and build information")

    # clean
    subparsers.add_parser("clean", help="Remove build artifacts")

    args = parser.parse_args()

    if args.command == "build":
        return cmd_build(args)
    elif args.command == "flash":
        return cmd_flash(args)
    elif args.command == "verify":
        return cmd_verify(args)
    elif args.command == "info":
        return cmd_info(args)
    elif args.command == "clean":
        return cmd_clean(args)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
