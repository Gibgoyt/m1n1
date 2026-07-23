# RUN 22 — patched m1n1: run the T602X LTSSM-enable writes in-window on the t8132 path

## Context

m4-pcie bring-up in `AppleSiliconM4/` (Mac16,10 / J773g / t8132) to enumerate the 1GbE NIC
(`lan-1gb`) on `pci-bridge2`. `pcie_init()` returns 0; both ports reach **LINKSTS BUSY**
(port0 `0x8300020c`, port2 `0x83000204`) and never reach UP. The phy-ip tunables axis is closed
(RUN 17). RUNs 18-21 attacked link training from the host (no reflash) and **all failed**:

- **RUN 18** (CLKREQ# → ADT alt-func 2): no change.
- **RUN 19** (per-port REFCLK REQ→ACK handshake): handshake succeeds, no change.
- **RUN 20** (replay m1n1's T602X LTSSM kick post-init): `ltssm+0x10/0x1c/0x20` latch, but
  **`ltssm+0x14` (LTSSM_START) reads back 0**; `rc_base+0x3c` and `port+0x10` also drop.
- **RUN 21** (refclk-first PERST# re-sequence, then write+read `ltssm+0x14`): re-sequence
  executed cleanly (PERST# GPIO toggled, `+0x82c` cycled, exc_count 0) but **`ltssm+0x14`
  still reads 0**, LINKSTS unchanged, every register identical to RUN 20, ECAM vacant. The
  **ordering hypothesis is falsified.**

### The mechanism (verified against m1n1 source + the RUN 20/21 logs)

The enrolled m1n1 (`/home/ahmed/Projects/C/embedded/m1n1`, HEAD `8a569ad`) maps t8132 →
`regs_t8140` (`pcie.c:411`, `type==APCIE_T8140`). Every register that refuses post-init Python
writes is **`APCIE_T602X`-gated** and never runs on our path:

- `set32(rc_base+0x3c, 0x1)` — the **arm** at `pcie.c:754` (gated `type==T602X`).
- `write32(port_base+0x10, 0x2)` at `758`.
- LTSSM kick `857-864` incl. `write32(ltssm+0x14, 0x1)` (gated `T602X && controller!=APCIE`).
- disarm `clear32(rc_base+0x3c, 0x1)` at `964` (gated `T602X||T6031`).

`rc_base+0x3c` bit0 is almost certainly a **write-enable / arm latch** for the port+ltssm
aperture: m1n1 arms it (754) before touching `port+0x10`/ltssm, disarms it (964) after. RUN
20/21 wrote those registers with the aperture **disarmed**, so they dropped.

**The no-reflash "arm it from Python first" idea is already falsified by the existing logs:**
RUN 20/21 seq-A did `set32(rc_base+0x3c, 0x1)` and it **read back `0x00000000`**
(`logs/21/nic-runtime.txt:720-721, 738-739`). Post-init, even the arm bit is write-locked
(so is `rc_base+0x024`). These writes must run **in-window during `pcie_init`**, at CPU speed
with the aperture armed — which only a patched m1n1 can do.

### Two traps a naive patch would hit (both must be handled)

1. **The idle-poll `continue`.** `pcie.c:866-869` polls `LINKSTS_BUSY→0` for 250 ms and, on
   timeout, `continue`s — *before* the "Do it again" LTSSM kick at `873-889`. On our machine
   BUSY never clears at 866, so a naive "enable the T602X kick" would still never reach it.
   The kick must be reachable even when the idle poll times out.
2. **The SError writes.** The T602X path is bundled with phy_ip/auspma/pll tunables (`594-651`)
   and PHY_CTRL writes (`648-673`: `set32(phy_base+4,0x10)`, `set32(phy_base+PHY_CTRL,0x300)`,
   the `phy_base+0x8` poll) that **AXI-stall / SError on j773g** (this is why `dc25f2f`
   reverted t8132 off `regs_t8122`). The patch must **exclude all of these**.

## Goal

A minimal, bisected patch to `m1n1/src/pcie.c` that, on the **t8132 path only** (guarded by
`adt_is_compatible(adt, adt_offset, "apcie,t8132")`, the same guard already used at `622`),
runs the T602X LTSSM-enable writes **in-window** — arm `rc_base+0x3c`, `port+0x10`, the LTSSM
kick incl. `ltssm+0x14=0x1` (reachable regardless of the idle-poll timeout), disarm — while
touching **none** of the phy_ip/PHY_CTRL SError writes. Each added write gets a `PCIE_BC`
breadcrumb so a wedge names its exact line. Rebuild, re-enroll, bump `--require-build`, and run.

## Approach

### 1. Patch `m1n1/src/pcie.c` — in-window T602X LTSSM enable, t8132-gated, bisected

Add a boolean local `bool t8132 = adt_is_compatible(adt, adt_offset, "apcie,t8132");` in
`pcie_init_controller()` (the guard string is already used at `622`). Then, gated on `t8132`
(NOT on flipping `type` to T602X — that would drag in the SError writes):

- **Arm + port+0x10** — mirror `pcie.c:753-759`, in the same per-port pre-bring-up slot:
  `set32(rc_base+0x3c, 0x1)` then (controller==APCIE) `write32(port_base+0x10, 0x2)`.
  `PCIE_BC` before/after each, reading back `rc_base+0x3c` so the log shows whether the arm
  latched in-window (the key new datum vs the post-init `0` we always see).
- **LTSSM kick, poll-independent** — after the STATUS_RUN poll (`851`) and the idle poll
  (`866`), but **arranged so the kick runs even if the idle poll would `continue`**: replicate
  the `873-889` body (`clear/set +0x82c` reset cycle → `udelay(1000)` →
  `write32(ltssm+0x10,0x2); write32(ltssm+0x1c,0x4); set32(ltssm+0x20,0x2);
  write32(ltssm+0x14,0x1)`) under `if (t8132)`, placed **before** the `866` idle poll's
  `continue` path can skip it (e.g. do the kick, then poll; or drop the `continue` for t8132
  and let the body run). `PCIE_BC` each write; read back `ltssm+0x14` — **the pivotal bit.**
- **Disarm** — mirror `pcie.c:964`: `clear32(rc_base+0x3c, 0x1)` under `if (t8132)`.
- **Exclude** everything in `594-651` and `648-673` (phy_ip/auspma/pll + PHY_CTRL writes).

**Bisection (one write-group per boot).** Use a small ADT-driven or compile toggle so the first
boot enables only the arm + `port+0x10` (and reads them back — does the arm latch in-window?),
the second adds the LTSSM kick, the third the disarm. Each boot flushes `PCIE_BC` breadcrumbs so
any wedge names the exact offending line (the RUN 13/17 wedge-localization discipline).

### 2. Rebuild + re-enroll + bump the build guard

- Build in the fork: `make` in `/home/ahmed/Projects/C/embedded/m1n1` (Makefile + `build/`
  already present); enroll via the project's normal chainload/kmutil path (`Scripts/m1n1/
  chainload.sh` / `smoke_test.sh` reference `build/kernel.bin`).
- **Bump `--require-build`** to the new `git describe` tag substring (currently `rc1-59-g`;
  the patch adds commits → new tag). This is mandatory — RUN 13 burned a boot on a stale
  binary because the guard still matched. Update the RUN 22 dispatcher arm's flag accordingly.

### 3. `Scripts/m1n1/perstn.py` — free read-only diagnostics + RUN 22 wiring

No new link-training writes needed from Python (the patch does the work). Add a small read-only
diagnostic bundle, gated behind `--endpoint-diag`, run once post-init:
- SMC key readback: `smc.smcep.read32("gP0d")` / `read32("gP1a")` (expect `0x800001` / `1`) to
  confirm endpoint fabric power stuck (reuse the SMC client already used by `smc_power`,
  `perstn.py:~281`).
- Decode + log the **port0-vs-port2 LINKSTS bit3 delta** (`0x8300020c` has bit3, `0x83000204`
  does not) — the one unexplained thread that could indicate endpoint presence/receiver-detect.
- Confirm the assigned MAC from the ADT (`lan-1gb`) is present (weak endpoint-present evidence).

This bundle is zero-write, wedge-immune, and interprets the patched boot's result (link trains
vs still BUSY vs endpoint-absent). The post-init `dump_pcie_regs` + `watch_linksts` + ECAM walk
already report the LINKSTS/`ltssm+0x14` outcome — reuse them.

### 4. New RUN 22 dispatcher arm — `Scripts/m1n1/perstn-run.sh`

Add `22)` before `*)` + `22` in the usage string. Flags: keep the RUN 18/19 posture, drop the
now-falsified post-init kick flags, add `--endpoint-diag`, and set the **new** `--require-build`
tag. The patched m1n1 does the LTSSM enable in-window during `pcie_init`; perstn.py just powers
on, runs pcie_init, diagnoses, and walks ECAM:

```
    22)
        # RUN 22: patched-m1n1 in-window T602X LTSSM enable. RUNs 18-21
        # exhausted the no-reflash axis: the T602X-gated writes (rc_base+0x3c
        # arm, port+0x10, ltssm+0x14 START) all refuse POST-init Python
        # writes -- RUN 20/21 seq-A even tried set32(rc_base+0x3c,0x1) and it
        # read back 0. They must run IN-WINDOW during pcie_init. This build
        # runs them on the t8132 path (excluding the phy_ip/PHY_CTRL SError
        # writes), with the kick reachable past the idle-poll continue.
        # REQUIRES the new patched build -- bump --require-build to its tag.
        FLAGS=(--preinit-probe
               --tier3
               --clkreq-mode=periph
               --setup-refclk=both
               --endpoint-diag
               --require-build=<NEW_BUILD_TAG>)
        ;;
```

## Critical files

- `/home/ahmed/Projects/C/embedded/m1n1/src/pcie.c` — the in-window patch: arm/`port+0x10`
  (~753-759), LTSSM kick reachable past the `866-869` idle-poll `continue` (mirror 873-889),
  disarm (~964), all `t8132`-gated + `PCIE_BC` breadcrumbed; exclude 594-651 / 648-673. Reg
  struct `regs_t8140` at 228-238; macros `APCIE_PORT_RESET_DIS`/`APCIE_T602X_PORT_RESET`
  (84-88); `LTSSM_START = ltssm+0x14` (861).
- `/home/ahmed/Projects/C/embedded/m1n1/` — `make` + enroll (Makefile, `build/`).
- `Scripts/m1n1/perstn.py` — `--endpoint-diag` read-only bundle (~5824 argparse; call post-init
  near the `watch_linksts`/ECAM section ~6278). Reuse `smc` client (~281), `_linksts_decode`.
- `Scripts/m1n1/perstn-run.sh` — new `22)` arm + usage; **bumped `--require-build`**.
- Reference only: `Scripts/m1n1/logs/21/nic-runtime.txt` (720-721/738-739 arm-read-0 proof;
  700-711 ltssm dump), `docs/ref-asahi-t8132-pcie.md` (SError history), `docs/project-m4-pcie-
  bringup.md`.

Follow-up (still owed): `Scripts/m1n1/logs/{18,19,20,21}/findings.md`.

## Verification

Real hardware (M4 mini + m1n1 over UART) — the user runs it:

1. **Build**: `make` in `/home/ahmed/Projects/C/embedded/m1n1` — confirm it compiles; note the
   new `git describe` tag for `--require-build`.
2. **Static (host tooling)**: `python3 -c "import ast; ast.parse(open('Scripts/m1n1/perstn.py'
   ).read())"`, `bash -n Scripts/m1n1/perstn-run.sh`, confirm `22)` routes.
3. **Enroll** the new build; verify the boot banner tag matches the bumped `--require-build`
   (the guard aborts on mismatch — RUN 13 lesson).
4. **Live**: `./Scripts/m1n1/perstn-run.sh 22` (bisect: first boot arm+port+0x10 only).
5. **Read** `/tmp/m4-recon/nic-runtime.txt` + the `TTY>` console, keying on:
   - **`PCIE_BC` breadcrumbs**: did the in-window `rc_base+0x3c` arm read back `0x1`
     (in-window latch — the thing Python couldn't do)? Did `ltssm+0x14` read back `0x1`?
   - the post-init `dump_pcie_regs` / `watch_linksts`: `BUSY CLEARED` vs `still BUSY`.
   - `--endpoint-diag`: SMC keys correct, MAC present, port0-vs-port2 bit3 delta.
   - if BUSY clears → ECAM walk prints the NIC VID:DID (class 0x02) — the goal.
6. **Outcome matrix** (record in `logs/21/findings.md`):
   - arm latches in-window + `ltssm+0x14=0x1` + BUSY clears → **link trains → NIC enumerates.**
   - arm latches, `ltssm+0x14=0x1`, BUSY persists → LTSSM kicked but no partner convergence →
     RUN 23 = longer link-up wait / per-port refclk-cgen / endpoint signal-integrity.
   - arm still reads 0 even in-window → the lock is deeper than the T602X arm (upstream
     analog/clock gate) → RUN 23 = cio3pllcore/pcieclkgen→rc_base in-C, or the analog-PLL axis.
   - a specific `PCIE_BC`'d write wedges (SError) → that exact line named → exclude it, re-bisect.
   - `--endpoint-diag` shows endpoint absent/unpowered → redirect off link-training entirely.
