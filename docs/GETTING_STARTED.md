# Getting started with KAST

This walks through setting KAST up from scratch on a printer that already homes X/Y with sensorless (StallGuard) homing, and already has some `driver_SGT` / homing current values that basically work. KAST's job is to make those values reliable and repeatable instead of guessed.

If you just want the quick reference (command params, config options), see the main [README](../README.md). This doc is more of a "do this, then this" walkthrough.

## 0. Before you start

- `install.sh` backs up your whole config folder automatically before it touches anything. If you're installing by hand instead, back up `printer.cfg` yourself first: `KAST_APPLY` writes to it via `SAVE_CONFIG`, same as any other Klipper calibration tool.
- Be near the printer, or watching a webcam, the first few times you run a sweep. Homing over and over at aggressive settings can occasionally overshoot before KAST catches it.
- Sensorless homing tuning is inherently a bit knock-y and buzzy on the steppers. That's normal.
- **Installing KAST changes how your printer homes.** It ships its own `[homing_override]`, which replaces any existing one entirely, not just for KAST commands, for every `G28` from then on. Before running any KAST command, send a plain `G28` by itself first, with your hand near an emergency stop, and watch it home from a cold start (Z not yet homed). Confirm the toolhead lifts before any X/Y move and doesn't hit anything, on both this and the very next `G28`, before you trust it unattended.

## 1. Run the installer

SSH into the host running klippy (the Pi, or wherever), and run:

```
wget -O - https://raw.githubusercontent.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning/main/install.sh | bash
```

It backs up your whole `printer_data/config` folder to `~/kast-backups/` before touching anything, then clones the repo to `~/kast`, symlinks `kast.py` into Klipper's `extras/` folder, symlinks the macros into your config folder and adds the `[include]` line for them, installs matplotlib into Klipper's venv (for auto-plotting), and registers KAST with Moonraker's update manager. It's safe to re-run any time, e.g. to pick up updates (each run makes a fresh backup too).

If your install doesn't follow the usual MainsailOS/Fluidd layout, override paths with environment variables, e.g. `KLIPPER_DIR=/home/pi/klipper wget -O - .../install.sh | bash`. See the top of `install.sh` for all of them.

Prefer doing it manually, or the installer doesn't fit your setup? Expand the steps below.

<details>
<summary>Manual install steps</summary>

Clone the whole repo somewhere inside your Klipper config tree (don't just copy `kast.py` on its own, auto-plotting looks for `scripts/kast_plot.py` relative to `kast.py`'s own location, so it needs the folder structure intact):

```
cd ~/printer_data/config
git clone https://github.com/DonaldLSayers/Klipper-Automated-Sensorless-Tuning.git kast
```

Symlink the extras module in:

```
ln -s ~/printer_data/config/kast/klippy/extras/kast.py ~/klipper/klippy/extras/kast.py
```

Add to `printer.cfg`:

```
[include kast/macros/kast.cfg]
```

For auto-plotting, install matplotlib into Klipper's venv:

```
~/klippy-env/bin/pip install matplotlib
```

(paths depend on your install). If you skip this, KAST still writes the raw CSV every time, you can graph it later from any machine with matplotlib installed.

</details>

## 2. Point the macros at your drivers, and read this if you already had a homing_override

Open `~/kast/macros/kast.cfg` and check `variable_driver_x` / `variable_driver_y` in `_KAST_HOMING_STATE` match your actual driver sections (e.g. `'tmc2209 stepper_x'` instead of the `'tmc2240 stepper_x'` default).

Also check `variable_prelift_z` (default 4mm). KAST's `[homing_override]` lifts Z this far before any X/Y homing move, using `SET_KINEMATIC_POSITION` to fake a known Z if Z hasn't been homed yet this session. This exists because a sensorless X/Y homing move at low Z can crash the nozzle into a bed clip, corner post, or the bed itself. Set it to whatever clears your printer's obstructions, or `0` if your printer genuinely has none (tall gantry, nothing in the way at Z=0). Don't guess low here, it's cheap insurance.

**If you already have your own `[homing_override]`**, whether that's a CoreXY dual-motor macro or anything else, **read this carefully before installing**. Klipper only allows one active `[homing_override]`, whichever one loads last fully replaces the other, there's no merging. That means every safety behavior your existing override has (Z lifts, park moves, obstruction avoidance, whatever it does) needs to be ported into `_KAST_HOME_AXIS` / `_KAST_HOME_Z` / the `[homing_override]` gcode in `kast.cfg`, not just the current-handling bits. Skipping a step here doesn't fail loudly, it fails as a crash the next time you home. Go through your existing macro line by line and make sure every move it makes has an equivalent in KAST's version before you trust it unattended.

## 3. Add the [kast] config section

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

## 4. Restart Klipper

```
RESTART
```

Check the console for errors. If `[kast]` fails to load, it's almost always a driver name mismatch (see step 2).

## 5. Check your baseline first

Before sweeping anything, see how your existing settings actually perform:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

This homes X ten times at whatever `driver_SGT` and current are already configured, and reports success rate and repeatability. If it's already at 100%, great, you might not need a full sweep. If it's flaky, that's your starting point.

## 6. Run a calibration sweep

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

## 7. Look at the graph

If auto-plotting is on, check `results_dir/stepper_x/` for the PNG. It plots score, success rate, and roughness (if you have an ADXL345) against SGT, one line per current/speed combo tested. You're looking for a wide flat region of 100% success rate with a low score, not just the single best point, since that's the more mechanically robust choice.

## 8. Apply and save

```
KAST_APPLY STEPPER=stepper_x
KAST_APPLY STEPPER=stepper_y
SAVE_CONFIG
```

This stages the winning values into your config (`driver_SGT` in the driver section, `homing_speed` in the stepper section if swept, `variable_home_current` in the macro if current was swept) and restarts Klipper to pick them up.

## 9. Re-check

After the restart, confirm it stuck:

```
KAST_TEST STEPPER=stepper_x AXIS=x SAMPLES=10
```

Should read 100% now. Worth re-running `KAST_TEST` again after any mechanical changes (belt tension, new hardware, etc.), since sensorless homing reliability can drift.
