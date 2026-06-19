---
name: run-seekfree-cyt4bb
description: Build, flash, debug, and monitor the Seekfree CYT4BB quadcopter flight controller firmware (IAR Embedded Workbench)
metadata:
  type: project
  unit: Seekfree CYT4BB 开源库
---

# Run: Seekfree CYT4BB 四轴飞控固件

This skill drives the **Seekfree CYT4BB Opensource Library** — a quadcopter
flight controller firmware for the Infineon Traveo T2G CYT4BB7CEE dual-core
ARM Cortex-M7 MCU (CM7_0: flight control, CM7_1: vision processing).

**Primary driver:** [.claude/skills/run-seekfree-cyt4bb/driver.py](.claude/skills/run-seekfree-cyt4bb/driver.py)
(also usable directly with IARBuild for more control).

Paths in this file are relative to the **repo root** (`Seekfree_CYT4BB_Opensource_Library/`).

## Prerequisites

- **IAR Embedded Workbench for ARM 9.2** (tested with 9.2.2.10770)
- **Windows** (IAR toolchain is Windows-only)
- **Hardware debug probe** (J-Link, I-jet, or CMSIS-DAP — configured in the .ewd file)
- **Target hardware** (the quadcopter flight controller board)

## Build

### Quick build (both cores)

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py build
```

This builds both CM7_0 (flight) and CM7_1 (vision) in sequence.

### Per-core build

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py build --core 7_0
python .claude/skills/run-seekfree-cyt4bb/driver.py build --core 7_1
```

### Direct IARBuild (equivalent, faster in CI)

```bash
# CM7_0 (flight controller)
"C:\Program Files\IAR Systems\Embedded Workbench 9.2\common\bin\iarbuild.exe" \
  project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug

# CM7_1 (vision processor)
"C:\Program Files\IAR Systems\Embedded Workbench 9.2\common\bin\iarbuild.exe" \
  project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

### Build outputs

| Core | File | Size |
|------|------|------|
| CM7_0 | `project/iar/project_config/Debug_m7_0/Exe/cyt4bb7_cm_7_0.out` | ~2 MiB |
| CM7_0 | `project/iar/project_config/Debug_m7_0/Exe/cyt4bb7_cm_7_0.hex` | ~142 KiB |
| CM7_1 | `project/iar/project_config/Debug_m7_1/Exe/cyt4bb7_cm_7_1.out` | ~1.4 MiB |
| CM7_1 | `project/iar/project_config/Debug_m7_1/Exe/cyt4bb7_cm_7_1.hex` | ~56 KiB |

### Verify build

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py verify
```

## Flash

**Requires a connected debug probe and target hardware.** The debugger
settings in `cyt4bb7_cm_7_0.ewd` / `cyt4bb7_cm_7_1.ewd` define the probe
type and flash loader (currently configured for the Infineon CYT4BB
device file at `$TOOLKIT_DIR$\config\debugger\Infineon\CYT4BB_M7.ddf`).

### Flash both cores (driver)

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py flash
```

### Flash via IAR C-SPY (direct)

```bash
# Flash CM7_0 (includes CM7_1 via multi-core debug config)
"C:\Program Files\IAR Systems\Embedded Workbench 9.2\common\bin\CSpyBat.exe" \
  project/iar/project_config/cyt4bb7_cm_7_0.ewd \
  project/iar/project_config/Debug_m7_0/Exe/cyt4bb7_cm_7_0.out \
  --flash_loader
```

The dual-core debug session (`cm7_0_cm7_1_debug.xml`) launches CM7_0 first,
then attaches to CM7_1. CM7_1 is a secondary core that starts after CM7_0
releases it from reset.

## Debug Serial Monitor

The flight controller outputs debug data at **20 Hz** over UART. Connect
to the board's UART (typically USB-serial on the flight controller board)
at **115200 baud** (check the UART peripheral configuration in your
board-specific code — the debug output uses `printf` from the IAR
semihosting or UART driver).

The debug print format is documented in the README:
```
UPF FA:%d valid:%d fresh:%d H:%.1f Vz:%.1f ...
```

## Project Info

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py info
```

## Clean

```bash
python .claude/skills/run-seekfree-cyt4bb/driver.py clean
```

This removes all `Debug_m7_0/` / `Debug_m7_1/` build directories. Rebuild
with `python driver.py build`.

## IAR IDE

The IAR workspace is at `project/iar/cyt4bb7.eww`, containing both core
projects. Open it in IAR Embedded Workbench for ARM 9.2+ to use the IDE
debugger with full register/memory views and dual-core debugging.

## Gotchas

- **Build order matters.** CM7_1 depends on the IPC shared-memory header
  (`cam_share.h`) which is included by both cores. Build CM7_0 first if
  there are linker script changes, though independent builds work.
- **Dual-core flash.** The CSpyBat flash-commander approach flashes both
  cores sequentially. The multicore debug XML (`cm7_0_cm7_1_debug.xml`)
  launches both C-SPY sessions — one per core — with CM7_1 in
  `attachToRunningTarget` mode so it waits for CM7_0 to release the
  secondary core from reset.
- **Linker script.** `project/iar/icf/linker_directives_tviibh.icf`
  defines the memory map. If flash addresses change, both .ewp files'
  linker config settings must match.
- **CYT4BB specific.** The device file (`CYT4BB_M7.ddf`) must match the
  exact silicon revision. If you swap MCU steppings, update the .ewd's
  `MemFile` setting.

## Troubleshooting

### Error: "Total number of errors: N" during build
Check the IAR compiler messages preceding the error count for the specific
file and line. Common causes:
- Syntax error in recently edited source
- Missing include path (check the .ewp file's compiler preprocessor settings)
- Outdated IAR version (9.2.2.10770 required)

### CM7_1 build succeeds, CM7_0 fails
IPC-related issues only affect CM7_0 (it reads the mailbox). Check
`cam_share.h` for struct layout changes.

### Warning: Pe177 "variable was declared but never referenced"
This is expected for unused debug variables. One instance in CM7_0
(`az` in `UP_FLOW_302.c` at line 160). Not a problem.

### CSpyBat flash fails — "Failed to connect to target"
- Check the debug probe is connected
- Verify the target board is powered
- Check the .ewd file's debugger driver setting matches your probe
- Try manually in IAR IDE: Project → Debug → check the device is detected

### Build outputs not found after build
The output directories are:
- `project/iar/project_config/Debug_m7_0/Exe/`
- `project/iar/project_config/Debug_m7_1/Exe/`

If the build completed but outputs are missing, check the IAR build log for
"Errors: 0". The build output path is set in the .ewp General Options →
Output → Executable.
