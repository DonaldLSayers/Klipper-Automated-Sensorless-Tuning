# Getting started with KAST

This walks through setting KAST up from scratch on a printer that already homes X/Y with sensorless (StallGuard) homing, and already has some `driver_SGT` / homing current values that basically work. KAST's job is to make those values reliable and repeatable instead of guessed.

If you just want the quick reference (command params, config options), see the main [README](../README.md). This doc is more of a "do this, then this" walkthrough.

## 0. Before you start

- Back up your `printer.cfg`. `KAST_APPLY` writes to it via `SAVE_CONFIG`, same as any other Klipper calibration tool, but it's still worth having a copy.
- Be near the printer, or watching a webcam, the first few times you run a sweep. Homing over and over at aggressive settings can occasionally overshoot before KAST catches it.
- Sensorless homing tuning is inherently a bit knock-y and buzzy on the steppers. That's normal.

## 1. Get the repo onto the printer

SSH into the host running klippy (the Pi, or wherever), and clone the whole repo somewhere inside your Klipper config tree, for example:

```
cd ~/printer_data/config
git clone https://github.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning.git kast
```

Don't just copy `kast.py` on its own. Auto-plotting looks for `scripts/kast_plot.py` relative to `kast.py`'s own location, so it needs the folder structure intact.

## 2. Symlink the extras module in

```
ln -s ~/printer_data/config/kast/klippy/extras/kast.py ~/klipper/klippy/extras/kast.py
```

Adjust paths to match your install. Using a symlink instead of a copy means `git pull` inside `kast/` keeps it updated.

## 3. Include the macros

In your `printer.cfg`:

```
[include kast/macros/kast.cfg]
```

This pulls in `_KAST_HOME_AXIS`, KAST's own `[homing_override]`, and the `KAST_TUNE_*` / `KAST_TEST_*` shortcut macros.

**If you already have your own `[homing_override]`** (a lot of CoreXY sensorless setups do, since homing one axis moves both motors), don't include both. Open `kast/macros/kast.cfg`, and merge your macro's per-axis logic into `_KAST_HOME_AXIS` there instead. The important part to keep is that it reads `s.home_current` from the shared `_KAST_HOMING_STATE` variable rather than a hardcoded number, that's what lets KAST's current sweep actually take effect instead of being silently overwritten right before each homing move.

Then check `variable_driver_x` / `variable_driver_y` in `_KAST_HOMING_STATE` match your actual driver sections (e.g. `'tmc2209 stepper_x'` instead of the `'tmc2240 stepper_x'` default).

## 4. Add the [kast] config section

```
[kast]
accel_chip: default
samples: 5
sgt_step: 8
fun_mode: true
results_dir: ~/printer_data/config/kast_results
enable_plots: true
```

See [docs/example-printer.cfg](example-printer.cfg) for the full annotated version. If you don't have an ADXL345 wired up, that's fine, leave `accel_chip` as-is and KAST will just skip the vibration scoring.

## 5. (Optional) enable auto-plotting

If you want KAST to render a PNG graph automatically after each calibration, matplotlib needs to be importable by whatever `python3` klippy resolves to on the host:

```
~/klippy-env/bin/pip install matplotlib
```

(path depends on your install, that's the typical MainsailOS/venv layout). If you skip this, KAST still writes the raw CSV every time, you can graph it later from any machine with matplotlib installed.

## 6. Restart Klipper

```
RESTART
```

Check the console for errors. If `[kast]` fails to load, it's almost always a driver name mismatch (see step 3).

## 7. Check your baseline first

Before sweeping anything, see how your existing settings actually perform:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

This homes X ten times at whatever `driver_SGT` and current are already configured, and reports success rate and repeatability. If it's already at 100%, great, you might not need a full sweep. If it's flaky, that's your starting point.

## 8. Run a calibration sweep

Start with just SGT:

```
KAST_CALIBRATE STEPPER=stepper_x AXIS=x
```

KAST will print an estimated number of homing moves before it starts (worth glancing at, a wide sweep with high SAMPLES can take a while). It homes repeatedly across the SGT range, and if an ADXL345 is set up, folds in vibration data too.

Once it's done, it reports the best SGT found and where the CSV/PNG landed. If you also want to sweep current or homing speed:

```
KAST_CALIBRATE STEPPER=stepper_x AXIS=x CURRENT_MIN=0.35 CURRENT_MAX=0.6 CURRENT_STEP=0.05
```

Adding more dimensions multiplies the trial count fast, so widen gradually rather than throwing a huge range at it on the first try.

Repeat for Y (and any other sensorless axis):

```
KAST_CALIBRATE STEPPER=stepper_y AXIS=y
```

Or just run `KAST_TUNE_ALL` to do both back to back.

## 9. Look at the graph

If auto-plotting is on, check `results_dir/stepper_x/` for the PNG. It plots score, success rate, and roughness (if you have an ADXL345) against SGT, one line per current/speed combo tested. You're looking for a wide flat region of 100% success rate with a low score, not just the single best point, since that's the more mechanically robust choice.

## 10. Apply and save

```
KAST_APPLY STEPPER=stepper_x
KAST_APPLY STEPPER=stepper_y
SAVE_CONFIG
```

This stages the winning values into your config (`driver_SGT` in the driver section, `homing_speed` in the stepper section if swept, `variable_home_current` in the macro if current was swept) and restarts Klipper to pick them up.

## 11. Re-check

After the restart, confirm it stuck:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

Should read 100% now. Worth re-running `KAST_TEST` again after any mechanical changes (belt tension, new hardware, etc.), since sensorless homing reliability can drift.
