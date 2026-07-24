# KAST — Klipper Automated Sensorless Tuning

KAST is a [Klipper](https://www.klipper3d.org/) `extras` module that
automatically searches for a reliable `driver_SGT` (StallGuard
sensitivity) and, optionally, homing current and homing speed for
sensorless-homing steppers — instead of hand-tuning by trial and
error.

It works by repeatedly homing an axis across a range of candidate
values and scoring each one on:

- **Reliability** — did the axis home successfully every time?
- **Repeatability** — how consistent is the triggered position across
  attempts?
- **Smoothness** *(optional)* — if an ADXL345 accelerometer is
  configured, KAST samples vibration during each homing move and
  penalizes candidates that look mechanically rough (a sign of near-miss
  stall detection or skipped steps), even if they technically "worked".

The best-scoring combination is reported, and can be staged into your
config with `KAST_APPLY` + `SAVE_CONFIG`.

## Status

Early / experimental. Test on a machine you can supervise — sweeping
`driver_SGT` and current will home the axis many times in a row.

## Install

Clone or symlink the **whole repo** into your Klipper config tree
(e.g. `~/printer_data/config/kast`), rather than copying just
`kast.py` — auto-plotting (below) locates `scripts/kast_plot.py`
relative to `kast.py`'s own path and won't find it otherwise.

1. Symlink `klippy/extras/kast.py` into your Klipper install's
   `klippy/extras/` directory.
2. `[include]` `macros/kast.cfg` from your `printer.cfg`. This installs
   KAST's own `[homing_override]` — if you already have one (e.g. a
   hand-written CoreXY dual-motor homing macro), merge its per-axis
   logic into `_KAST_HOME_AXIS` in `macros/kast.cfg` instead of
   including both, or `KAST_CALIBRATE`'s current/speed sweeps will get
   silently overridden by your macro's own hardcoded values right
   before each homing move.
3. In `macros/kast.cfg`, set `variable_driver_x` / `variable_driver_y`
   in `_KAST_HOMING_STATE` to your actual TMC driver sections (e.g.
   `'tmc2209 stepper_x'`) if they aren't TMC2240.
4. Add a `[kast]` section — see [docs/example-printer.cfg](docs/example-printer.cfg).
5. For auto-plotting, install matplotlib somewhere `python3` on the
   host can see it (`pip install matplotlib` in Klipper's venv, or
   system-wide). Not required otherwise — KAST still writes CSVs.
6. Restart Klipper (`RESTART`).

## Usage

```
KAST_CALIBRATE STEPPER=stepper_x AXIS=x
```

Optional parameters:

| Param | Default | Meaning |
|---|---|---|
| `AXIS` | last char of `STEPPER` | Axis letter to home (`x`/`y`/`z`) |
| `SGT_MIN` / `SGT_MAX` | driver-dependent | Sweep range for the stall-sensitivity field. `-64`/`63` for signed `sgt` drivers (tmc2130/2660/5160/2240), `0`/`255` for unsigned `sg4_thrs` drivers (tmc2208/2209/2226) |
| `SGT_STEP` | `8` (config `sgt_step`) | Step size across the sweep |
| `SAMPLES` | `5` (config `samples`) | Homing attempts per candidate |
| `CURRENT_MIN` / `CURRENT_MAX` | unset | Also sweep homing current (amps) over this range |
| `CURRENT_STEP` | `0.1` | Step size for the current sweep |
| `HOMING_SPEED_MIN` / `HOMING_SPEED_MAX` | unset | Also sweep homing speed (mm/s) over this range |
| `HOMING_SPEED_STEP` | `5.0` | Step size for the speed sweep |

StallGuard sensitivity is velocity-dependent — an SGT that's reliable
at one homing speed may not be at another — so if you care about a
specific `homing_speed`, it's worth sweeping SGT at that speed rather
than assuming a value tuned elsewhere carries over. Sweeping all three
dimensions (SGT × current × speed) multiplies trial count fast; KAST
prints an estimated homing-move count before starting.

Then:

```
KAST_STATUS                     # show last result(s)
KAST_APPLY STEPPER=stepper_x    # stage best values into saved config
SAVE_CONFIG                     # write + restart
```

`macros/kast.cfg` also provides `KAST_TUNE_X`, `KAST_TUNE_Y`, and
`KAST_TUNE_ALL` as shortcuts.

## Testing current settings

To sanity-check whatever is *currently* configured — no sweeping, no
changes — without running a full calibration:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

Useful after `SAVE_CONFIG`, after mechanical changes, or just to
confirm a config is still reliable. `macros/kast.cfg` provides
`KAST_TEST_X`, `KAST_TEST_Y`, and `KAST_TEST_ALL` shortcuts.

## Notes on TMC drivers

KAST auto-detects the TMC driver behind each stepper and uses the
correct StallGuard field:

- TMC2130 / TMC2660 / TMC5160 / TMC2240 → `sgt` (signed, -64 to 63, config `driver_SGT`)
- TMC2208 / TMC2209 / TMC2226 → `sg4_thrs` (unsigned, 0 to 255, config `driver_SGTHRS`)

On TMC2240, `sgt` drives the default SpreadCycle-based sensorless
homing path. TMC2240 also has a separate `sg4_thrs` field that only
takes effect if you've deliberately switched to SG4/StealthChop-based
homing (non-zero `sg4_thrs`); KAST does not target that mode.

Values are applied live via Klipper's built-in `SET_TMC_FIELD` /
`SET_TMC_CURRENT` commands, so no driver-specific code lives in KAST
itself. Homing speed has no equivalent gcode command — Klipper fixes
it at config-parse time — so KAST overrides it by writing directly to
the axis's `PrinterRail` object in memory for the duration of a sweep,
then restores the original value.

## What KAST_APPLY persists, and where

- `driver_SGT` / `driver_SGTHRS` (the field KAST tuned) → the driver's
  own config section, e.g. `[tmc2240 stepper_x]`.
- `homing_speed`, if swept → the stepper's section, e.g. `[stepper_x]`
  (this is a real Klipper config option).
- Homing current, if swept → `variable_home_current` in
  `[gcode_macro _KAST_HOMING_STATE]`, since Klipper has no native
  `home_current` config field. If you're not using `macros/kast.cfg`'s
  homing override, KAST reports the best current instead and you apply
  it by hand to wherever your own macro sets it.

## Results and graphs

Every `KAST_CALIBRATE` run writes a CSV of all trials to
`results_dir/<stepper_name>/kast_<stepper_name>_<timestamp>.csv`
(default `results_dir`: `~/printer_data/config/kast_results`) — one
subfolder per stepper, similar to how
[klippain-shaketune](https://github.com/Frix-x/klippain-shaketune)
organizes its resonance-test output.

If `enable_plots` is on (default) and matplotlib is available, KAST
launches `scripts/kast_plot.py` as a detached background process right
after writing the CSV — it never blocks klippy's reactor, and a
missing/slow matplotlib just means no PNG shows up, with the CSV kept
either way. The PNG lands next to the CSV and plots score, success
rate, and (if an ADXL345 was used) roughness, all against SGT, with
one line per current/speed combination swept.

To render a graph manually (e.g. from a workstation, or if
auto-plotting is off):

```
python3 scripts/kast_plot.py results_dir/stepper_x/kast_stepper_x_1234567890.csv
```

## ADXL345 (optional)

If a `[adxl345]` (or `[adxl345 <name>]`) section exists, set
`accel_chip` in `[kast]` accordingly (`default` for the unnamed
section). KAST will fold vibration data into its scoring automatically.
Without an accelerometer, KAST still works — it just scores purely on
homing success/repeatability.

## Fun mode

Long sweeps get chatty status messages by default (set `fun_mode:
false` in `[kast]` to turn this off).

## License

GPLv3 — see [LICENSE](LICENSE). Matches Klipper's own license.
