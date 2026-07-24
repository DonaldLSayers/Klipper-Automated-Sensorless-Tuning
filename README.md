# KAST - Klipper Automated Sensorless Tuning

KAST is a [Klipper](https://www.klipper3d.org/) `extras` module that automatically searches for a reliable `driver_SGT` (StallGuard sensitivity), and optionally homing current and homing speed, for sensorless-homing steppers. No more hand-tuning by trial and error.

It works by repeatedly homing an axis across a range of candidate values and scoring each one on:

- **Reliability**: did the axis home successfully every time?
- **Repeatability**: how consistent is the triggered position across attempts?
- **Smoothness** *(optional)*: if an ADXL345 accelerometer is configured, KAST samples vibration during each homing move and penalizes candidates that look mechanically rough (a sign of near-miss stall detection or skipped steps), even if they technically "worked".

The best-scoring combination gets reported, and can be staged into your config with `KAST_APPLY` + `SAVE_CONFIG`.

New here? [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) walks through the whole process step by step. This README is more of a reference.

## Status

Early / experimental. Test on a machine you can supervise, sweeping `driver_SGT` and current will home the axis many times in a row.

## Install

SSH into the machine running klippy and run:

```
wget -O - https://raw.githubusercontent.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning/main/install.sh | bash
```

First thing it does is tar up your whole `printer_data/config` folder to `~/kast-backups/`, before touching anything. Then it clones the repo to `~/kast`, symlinks `kast.py` into Klipper's `extras` folder, symlinks the macros into your config and adds the `[include]` line for them, installs matplotlib into Klipper's venv for auto-plotting, and registers KAST with Moonraker's update manager so it shows up for updates in Mainsail/Fluidd. Safe to re-run any time.

It won't touch your `printer.cfg` beyond that one include line. You still need to:

1. Check `variable_driver_x` / `variable_driver_y` in `~/kast/macros/kast.cfg`'s `_KAST_HOMING_STATE` match your actual TMC driver sections (e.g. `'tmc2209 stepper_x'`) if they aren't TMC2240.
2. If you already have your own `[homing_override]` (common on CoreXY sensorless setups), merge its logic into `_KAST_HOME_AXIS` instead of running both, see [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) for why.
3. Add a `[kast]` section, see [docs/example-printer.cfg](docs/example-printer.cfg).
4. Restart Klipper (`RESTART`).

Prefer to do it by hand, or the installer doesn't fit your setup? The full manual steps are in [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).

## Usage

```
KAST_CALIBRATE STEPPER=stepper_x AXIS=x
```

Optional parameters:

| Param | Default | Meaning |
|---|---|---|
| `AXIS` | last char of `STEPPER` | Axis letter to home (`x`/`y`/`z`) |
| `SGT_MIN` / `SGT_MAX` | see below | Sweep range for the stall-sensitivity field |
| `SGT_RADIUS` | `16` (config `sgt_radius`) | How far past the currently configured SGT the default sweep explores, in the more-sensitive direction only |
| `SGT_STEP` | `8` (config `sgt_step`) | Step size across the sweep |
| `SAMPLES` | `5` (config `samples`) | Homing attempts per candidate |
| `CURRENT_MIN` / `CURRENT_MAX` | unset | Also sweep homing current (amps) over this range |
| `CURRENT_STEP` | `0.1` | Step size for the current sweep |
| `HOMING_SPEED_MIN` / `HOMING_SPEED_MAX` | unset | Also sweep homing speed (mm/s) over this range |
| `HOMING_SPEED_STEP` | `5.0` | Step size for the speed sweep |

If `driver_SGT`/`driver_SGTHRS` is already set in printer.cfg, the default sweep only explores toward *more* sensitive settings from there, never less. A known-working value means the printer already survives that much force, testing something harsher has no upside, only risk of a worse impact. Concretely: for signed `sgt` drivers (lower = more sensitive), the default range is `current - SGT_RADIUS` to `current`, it never defaults past `current`. For unsigned `sg4_thrs` drivers (higher = more sensitive), it's the same idea in reverse, `current` to `current + SGT_RADIUS`. Pass `SGT_MIN`/`SGT_MAX` explicitly if you deliberately want to test the harsher direction too. If `driver_SGT`/`driver_SGTHRS` isn't set yet (a fresh printer with no baseline), KAST falls back to the driver's full theoretical range, since there's nothing to anchor to yet, that first run is the one to watch closely.

When a baseline exists, KAST doesn't blindly sweep the whole computed range either. It tests the baseline value first, then steps one `SGT_STEP` at a time away from it, stopping in that direction the moment a step isn't 100% reliable, rather than committing to the whole range regardless of what happens along the way.

That alone isn't enough, though: it only catches a step that fails outright. It cannot catch a step that "succeeds" via a false, early StallGuard trigger, since Klipper reports the same configured `position_endstop` value after any triggered `G28` whether the trigger was real or not, there's nothing in that reported position to tell the two apart. The real protection against that is in `macros/kast.cfg`'s `_KAST_HOME_AXIS`, which measures how far the toolhead actually physically moved between just before and just after each `G28` (not relying on Klipper's reported position at all) and aborts the entire calibration immediately, before the backoff move that would otherwise turn a false trigger into real drift, if a homing move looks too short to be genuine (`variable_min_home_travel`, default 20mm, sized to roughly the real travel distance a working home in this macro should cover).

The same principle applies if you sweep `CURRENT_MIN`/`CURRENT_MAX`: higher current means more torque before StallGuard triggers, which generally means a harder impact. There's no automatic default here since current sweeping is opt-in, but it's worth keeping `CURRENT_MAX` at or below whatever's already working rather than pushing it higher, for the same reason as the SGT default above.

StallGuard sensitivity is velocity-dependent, an SGT that's reliable at one homing speed may not be at another. So if you care about a specific `homing_speed`, it's worth sweeping SGT at that speed rather than assuming a value tuned elsewhere carries over. Sweeping all three dimensions (SGT x current x speed) multiplies trial count fast, KAST prints an estimated homing-move count before starting so you know what you're in for.

Then:

```
KAST_STATUS                     # show last result(s)
KAST_APPLY STEPPER=stepper_x    # stage best values into saved config
SAVE_CONFIG                     # write + restart
```

`macros/kast.cfg` also provides `KAST_TUNE_X`, `KAST_TUNE_Y`, and `KAST_TUNE_ALL` as shortcuts.

## Testing current settings

To sanity-check whatever is currently configured, no sweeping, no changes, without running a full calibration:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

Useful after `SAVE_CONFIG`, after mechanical changes, or just to confirm a config is still reliable. `macros/kast.cfg` provides `KAST_TEST_X`, `KAST_TEST_Y`, and `KAST_TEST_ALL` shortcuts.

## Notes on TMC drivers

KAST auto-detects the TMC driver behind each stepper and uses the correct StallGuard field:

- TMC2130 / TMC2660 / TMC5160 / TMC2240: `sgt` (signed, -64 to 63, config `driver_SGT`)
- TMC2208 / TMC2209 / TMC2226: `sg4_thrs` (unsigned, 0 to 255, config `driver_SGTHRS`)

On TMC2240, `sgt` drives the default SpreadCycle-based sensorless homing path. TMC2240 also has a separate `sg4_thrs` field that only takes effect if you've deliberately switched to SG4/StealthChop-based homing (non-zero `sg4_thrs`). KAST does not target that mode.

Values are applied live via Klipper's built-in `SET_TMC_FIELD` / `SET_TMC_CURRENT` commands, so no driver-specific code lives in KAST itself. Homing speed has no equivalent gcode command since Klipper fixes it at config-parse time, so KAST overrides it by writing directly to the axis's `PrinterRail` object in memory for the duration of a sweep, then restores the original value.

## What KAST_APPLY persists, and where

- `driver_SGT` / `driver_SGTHRS` (the field KAST tuned) goes into the driver's own config section, e.g. `[tmc2240 stepper_x]`.
- `homing_speed`, if swept, goes into the stepper's section, e.g. `[stepper_x]` (this is a real Klipper config option).
- Homing current, if swept, goes into `variable_home_current` in `[gcode_macro _KAST_HOMING_STATE]`, since Klipper has no native `home_current` config field. If you're not using `macros/kast.cfg`'s homing override, KAST just reports the best current instead and you apply it by hand wherever your own macro sets it.

## Results and graphs

Every `KAST_CALIBRATE` run writes a CSV of all trials to `results_dir/<stepper_name>/kast_<stepper_name>_<timestamp>.csv` (default `results_dir`: `~/printer_data/config/kast_results`), one subfolder per stepper, similar to how [klippain-shaketune](https://github.com/Frix-x/klippain-shaketune) organizes its resonance-test output.

If `enable_plots` is on (default) and matplotlib is available, KAST launches `scripts/kast_plot.py` as a detached background process right after writing the CSV. It never blocks klippy's reactor, and a missing/slow matplotlib just means no PNG shows up, the CSV is kept either way. The PNG lands next to the CSV and plots score, success rate, and (if an ADXL345 was used) roughness, all against SGT, with one line per current/speed combination swept.

To render a graph manually (e.g. from a workstation, or if auto-plotting is off):

```
python3 scripts/kast_plot.py results_dir/stepper_x/kast_stepper_x_1234567890.csv
```

## ADXL345 (optional)

If a `[adxl345]` (or `[adxl345 <name>]`) section exists, set `accel_chip` in `[kast]` accordingly (`default` for the unnamed section). KAST will fold vibration data into its scoring automatically. Without an accelerometer, KAST still works, it just scores purely on homing success/repeatability.

## Fun mode

Long sweeps get chatty status messages by default (set `fun_mode: false` in `[kast]` to turn this off).

## License

GPLv3, see [LICENSE](LICENSE). Matches Klipper's own license.
