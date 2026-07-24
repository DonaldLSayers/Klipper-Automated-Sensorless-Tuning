########################################################################
# KAST - Klipper Automated Sensorless Tuning
#
# Automatically searches for reliable driver_SGT (stall sensitivity)
# and home_current values on sensorless-homing steppers, using repeated
# homing trials and (optionally) ADXL345 vibration data as a quality
# signal.
#
# License: GPLv3
########################################################################

import csv
import logging
import os
import statistics
import subprocess
import sys
import time

# TMC field name that controls stall sensitivity, per driver family.
# TMC2130/TMC2660/TMC5160 use a signed "sgt" field, -64 to 63
# (config: driver_SGT). Lower = more sensitive/easier to trigger.
# TMC2208/TMC2209/TMC2226 use an unsigned "sg4_thrs" field, 0-255
# (config: driver_SGTHRS in some Klipper versions). Higher = more
# sensitive.
# TMC2240 supports BOTH: "sgt" (signed, SpreadCycle-based homing,
# the default and what most configs use) and "sg4_thrs" (unsigned,
# only active if sg4_thrs is set non-zero, which switches Klipper to
# SG4/StealthChop-based homing instead). KAST targets "sgt" for
# TMC2240 since that's the default homing path; if a printer has been
# deliberately switched to SG4 homing (sg4_thrs != 0), this module
# will tune the wrong field and needs adjusting by hand.
SGT_FIELD_BY_DRIVER = {
    'tmc2130': 'sgt',
    'tmc2660': 'sgt',
    'tmc5160': 'sgt',
    'tmc2208': 'sg4_thrs',
    'tmc2209': 'sg4_thrs',
    'tmc2226': 'sg4_thrs',
    'tmc2240': 'sgt',
}

SGT_SIGNED_DRIVERS = ('tmc2130', 'tmc2660', 'tmc5160', 'tmc2240')

# printer.cfg option name to persist the value under (KAST_APPLY),
# and the valid value range for that field.
CONFIG_KEY_BY_DRIVER = {
    'tmc2130': 'driver_SGT',
    'tmc2660': 'driver_SGT',
    'tmc5160': 'driver_SGT',
    'tmc2208': 'driver_SGTHRS',
    'tmc2209': 'driver_SGTHRS',
    'tmc2226': 'driver_SGTHRS',
    'tmc2240': 'driver_SGT',
}
SGT_RANGE_BY_DRIVER = {
    driver_type: ((-64, 63) if driver_type in SGT_SIGNED_DRIVERS
                  else (0, 255))
    for driver_type in SGT_FIELD_BY_DRIVER
}

# Fun, low-stakes console chatter so long sweeps don't feel silent.
BOOP_LINES = [
    "*boop* testing SGT=%s...",
    "poke poke -> SGT=%s",
    "beep! trying SGT=%s",
    "nudging the stepper with SGT=%s...",
    "here goes nothing, SGT=%s",
    "*bonk* SGT=%s, let's see...",
]


class KASTError(Exception):
    pass


class KASTUnsafeAbort(Exception):
    """Raised when a homing move looks physically unsafe to continue
    from (e.g. a suspiciously short move suggesting a false StallGuard
    trigger, see macros/kast.cfg's min_home_travel check). Unlike an
    ordinary failed homing attempt, this aborts the entire calibration
    immediately rather than being recorded as one failed sample and
    continuing on to the next trial."""
    pass


class KASTDriverAdapter:
    """Wraps SET_TMC_FIELD / SET_TMC_CURRENT gcode commands so KAST does
    not need to know about individual TMC driver module internals."""

    def __init__(self, printer, stepper_name):
        self.printer = printer
        self.stepper_name = stepper_name
        self.gcode = printer.lookup_object('gcode')
        self.driver_name, self.driver_type = self._find_driver()
        self.sgt_field = SGT_FIELD_BY_DRIVER.get(self.driver_type)
        if self.sgt_field is None:
            raise KASTError(
                "KAST: stepper '%s' uses driver '%s', which does not "
                "support sensorless homing (StallGuard)."
                % (stepper_name, self.driver_type))
        self.sgt_signed = self.driver_type in SGT_SIGNED_DRIVERS
        self.config_key = CONFIG_KEY_BY_DRIVER[self.driver_type]
        self.sgt_range = SGT_RANGE_BY_DRIVER[self.driver_type]

    def get_configured_sgt(self):
        """Returns the SGT value currently sitting in printer.cfg for
        this driver, or None if it was never set (e.g. still at the
        module's own default). Used to center the default sweep range
        on a known-working value instead of blindly walking the full
        theoretical range, which can mean testing much harsher
        settings than the printer has ever actually seen."""
        try:
            configfile = self.printer.lookup_object('configfile')
            section = configfile.validate.status_settings.get(
                self.driver_name.lower(), {})
            val = section.get(self.config_key.lower())
            if val is None:
                return None
            return int(float(val))
        except (AttributeError, TypeError, ValueError) as e:
            logging.warning(
                "KAST: could not read configured %s for '%s': %s "
                "(falling back to the driver's full sweep range)",
                self.config_key, self.stepper_name, e)
            return None

    def _find_driver(self):
        for driver_type in SGT_FIELD_BY_DRIVER:
            full_name = "%s %s" % (driver_type, self.stepper_name)
            obj = self.printer.lookup_object(full_name, None)
            if obj is not None:
                return full_name, driver_type
        raise KASTError(
            "KAST: no TMC driver found for stepper '%s'. Checked: %s"
            % (self.stepper_name,
               ", ".join(SGT_FIELD_BY_DRIVER.keys())))

    def set_sgt(self, value):
        self.gcode.run_script_from_command(
            "SET_TMC_FIELD STEPPER=%s FIELD=%s VALUE=%d"
            % (self.stepper_name, self.sgt_field, int(value)))

    def set_current(self, current):
        self.gcode.run_script_from_command(
            "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f"
            % (self.stepper_name, current))
        # Also sync KAST's shared homing-state variable, in case a
        # [homing_override] (e.g. macros/kast.cfg's _KAST_HOME_AXIS)
        # re-applies its own homing current right before G28 runs --
        # otherwise the SET_TMC_CURRENT above gets clobbered before it
        # ever takes effect. No-op if that macro isn't in use.
        if self.printer.lookup_object(
                'gcode_macro _KAST_HOMING_STATE', None) is not None:
            self.gcode.run_script_from_command(
                "SET_GCODE_VARIABLE MACRO=_KAST_HOMING_STATE "
                "VARIABLE=home_current VALUE=%.3f" % current)


class KASTHomingSpeedOverride:
    """Temporarily overrides a stepper's homing speed by reaching into
    its PrinterRail. Klipper has no gcode command for this (homing
    speed is normally fixed at config-parse time), so this pokes the
    in-memory rail object directly -- the same value Homing.home_rails()
    reads at G28 time. Only works for kinematics that expose `.rails`
    (cartesian, corexy, hybrid_corexy, etc.)."""

    def __init__(self, printer, stepper_name):
        self.rail = self._find_rail(printer, stepper_name)
        self.orig_speed = None

    @staticmethod
    def _find_rail(printer, stepper_name):
        toolhead = printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        for rail in getattr(kin, 'rails', []):
            for stepper in rail.get_steppers():
                if stepper.get_name() == stepper_name:
                    return rail
        raise KASTError(
            "KAST: could not find a homing rail for stepper '%s' "
            "(homing-speed sweeps need cartesian-style kinematics)."
            % stepper_name)

    def set(self, speed):
        if self.orig_speed is None:
            self.orig_speed = self.rail.homing_speed
        self.rail.homing_speed = speed

    def restore(self):
        if self.orig_speed is not None:
            self.rail.homing_speed = self.orig_speed
            self.orig_speed = None


class KASTSafeZoneTester:
    """Tests whether a given SGT/current causes a false StallGuard
    trigger during ordinary, unobstructed back-and-forth moves near
    bed center, instead of searching toward the real endstop. Since
    there's no hard stop anywhere near the test region, a false
    trigger just means the move stops early and gets logged, not a
    crash -- this is what makes it safe to explore SGT much more
    directly than a real homing search would be.

    Built on Klipper's own manual_home() (the same primitive probes
    use for probing_move()), with check_triggered=False: if the
    endstop never fires, the move just completes normally at the
    target with no error, rather than raising like a real homing
    search would after a full-travel failure.

    Important limitation: this only finds the SGT below which
    StallGuard does NOT false-trigger under ordinary (lower) load. It
    does not by itself confirm a value is sensitive enough to reliably
    trigger at the real endstop, where the mechanical stall applies
    more load than free motion does. In practice a value that's safe
    here should trigger at least as reliably there, but KAST_TEST's
    real G28 check is what actually confirms that before trusting a
    value for print use."""

    def __init__(self, printer, gcode, stepper_name, axis, travel_mm, speed):
        self.printer = printer
        self.gcode = gcode
        self.axis = axis.lower()
        self.axis_idx = {'x': 0, 'y': 1}[self.axis]
        self.travel_mm = travel_mm
        self.speed = speed
        self.toolhead = printer.lookup_object('toolhead')
        self.rail = KASTHomingSpeedOverride._find_rail(printer, stepper_name)
        self.endstops = self.rail.get_endstops()
        self.phoming = printer.lookup_object('homing')

    def _bed_center(self):
        eventtime = self.printer.get_reactor().monotonic()
        status = self.toolhead.get_status(eventtime)
        axis_min = status['axis_minimum']
        axis_max = status['axis_maximum']
        return (axis_min[self.axis_idx] + axis_max[self.axis_idx]) / 2.0

    def _move_to_center(self, center):
        self.gcode.run_script_from_command(
            "G90\nG1 %s%.3f F%d\nM400"
            % (self.axis.upper(), center, int(self.speed * 60)))

    def _test_one_direction(self, center, direction):
        """Returns (triggered, travelled_mm)."""
        target = list(self.toolhead.get_position())
        target[self.axis_idx] = center + direction * self.travel_mm
        epos = self.phoming.manual_home(
            self.toolhead, self.endstops, target, self.speed,
            probe_pos=False, triggered=True, check_triggered=False)
        travelled = abs(epos[self.axis_idx] - center)
        triggered = travelled < (self.travel_mm - 1.0)
        return triggered, travelled

    def test(self):
        """Runs one full back-and-forth cycle (both directions) from
        bed center. Returns (ok, err); ok is True only if NEITHER
        direction false-triggered."""
        center = self._bed_center()
        self._move_to_center(center)
        result_ok, result_err = True, None
        for direction in (1, -1):
            triggered, travelled = self._test_one_direction(
                center, direction)
            # Return to center regardless of outcome -- both a
            # trigger and a completed move land well within the safe
            # zone (bed center +/- travel_mm), so an ordinary move
            # back is safe either way.
            self._move_to_center(center)
            if triggered and result_ok:
                axis_label = ("+" if direction > 0 else "-") + self.axis.upper()
                result_ok, result_err = False, (
                    "false trigger testing %s: stopped after %.1fmm "
                    "of %.1fmm" % (axis_label, travelled, self.travel_mm))
        return result_ok, result_err


class KASTAccelHelper:
    """Optional ADXL345 vibration capture around a homing move. Produces
    a single 'roughness' score (higher = more vibration / more likely a
    skipped step or false trigger) so it can feed the search's scoring
    function. Silently disabled if no adxl345 chip is configured."""

    def __init__(self, printer, chip_name):
        self.printer = printer
        self.chip_name = chip_name
        self.chip = None
        if chip_name:
            full_name = 'adxl345' if chip_name == 'default' else \
                'adxl345 %s' % chip_name
            self.chip = printer.lookup_object(full_name, None)

    @property
    def enabled(self):
        return self.chip is not None

    def measure(self, action_fn):
        """Runs action_fn() while sampling the accelerometer. Returns
        (roughness_score, action_result). roughness_score is None if
        accel data could not be captured."""
        if not self.enabled:
            return None, action_fn()
        client = self.chip.start_internal_client()
        try:
            result = action_fn()
        finally:
            client.finish_measurements()
        if not client.has_valid_samples():
            return None, result
        samples = client.get_samples()
        if len(samples) < 4:
            return None, result
        mags = [
            (x * x + y * y + z * z) ** 0.5
            for _t, x, y, z in samples
        ]
        mean_mag = sum(mags) / len(mags)
        variance = sum((m - mean_mag) ** 2 for m in mags) / len(mags)
        roughness = variance ** 0.5
        return roughness, result


class KASTStepperTuner:
    """Runs the sweep/search for a single stepper (one axis / one motor)."""

    def __init__(self, kast, stepper_name, axis, config_section,
                 safe_zone_travel=None, safe_zone_speed=None):
        self.kast = kast
        self.printer = kast.printer
        self.gcode = kast.gcode
        self.stepper_name = stepper_name
        self.axis = axis
        self.driver = KASTDriverAdapter(self.printer, stepper_name)
        self.accel = kast.accel
        self._speed_override = None
        # safe_zone_travel not None selects the safe-zone tester (see
        # KASTSafeZoneTester) instead of real G28 homing for trial()'s
        # single-attempt test. Used by KAST_CALIBRATE; KAST_TEST keeps
        # using real G28, since confirming the CURRENT settings home
        # for real is the whole point of that command.
        self.safe_tester = None
        if safe_zone_travel is not None:
            self.safe_tester = KASTSafeZoneTester(
                self.printer, self.gcode, stepper_name, axis,
                safe_zone_travel, safe_zone_speed)
        self._reset_cycle_drift_baseline()

    def _reset_cycle_drift_baseline(self):
        """macros/kast.cfg's _KAST_CHECK_CYCLE_START compares each
        homing cycle's resting position against the previous cycle's,
        which is only meaningful across back-to-back calls within one
        run, not across unrelated G28s wherever the toolhead happens
        to be. Reset the baseline at the start of every run so it
        doesn't false-positive on normal use between runs."""
        if self.printer.lookup_object(
                'gcode_macro _KAST_HOMING_STATE', None) is None:
            return
        for var in ('last_cycle_start_x', 'last_cycle_start_y'):
            self.gcode.run_script_from_command(
                "SET_GCODE_VARIABLE MACRO=_KAST_HOMING_STATE "
                "VARIABLE=%s VALUE=-99999" % var)

    def boop(self, sgt):
        if self.kast.fun_mode:
            label = sgt if sgt is not None else "current settings"
            line = BOOP_LINES[self._boop_i % len(BOOP_LINES)] % label
            self._boop_i += 1
            self.gcode.respond_info(line)

    def celebrate(self, sgt, current):
        if self.kast.fun_mode:
            msg = "Found a good spot! SGT=%s" % sgt
            if current is not None:
                msg += " CURRENT=%.2fA" % current
            self.gcode.respond_info(msg + ". Nice.")

    _boop_i = 0

    def _set_homing_speed(self, speed):
        if speed is None:
            return
        if self._speed_override is None:
            self._speed_override = KASTHomingSpeedOverride(
                self.printer, self.stepper_name)
        self._speed_override.set(speed)

    def restore_homing_speed(self):
        if self._speed_override is not None:
            self._speed_override.restore()

    def _home_once(self):
        """Homes just this stepper's axis and reports whether it
        succeeded, plus the triggered position (for repeatability
        scoring)."""
        try:
            self.gcode.run_script_from_command(
                "G28 %s" % self.axis.upper())
        except self.gcode.error as e:
            msg = str(e)
            if 'KAST_UNSAFE_TRIGGER' in msg:
                raise KASTUnsafeAbort(msg)
            return False, None, msg
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        idx = {'x': 0, 'y': 1, 'z': 2}[self.axis.lower()]
        return True, pos[idx], None

    def _safe_zone_once(self):
        """Same (ok, pos, err) shape as _home_once, but via
        KASTSafeZoneTester: ok=True means no false trigger during the
        back-and-forth test move, not "homed successfully". pos is
        always None here, position isn't diagnostic in the same way
        when every attempt returns to the same bed-center reference.

        A gcode.error (e.g. "no trigger") is an ordinary, understood
        outcome and is recorded as a normal failed trial, the search
        just continues. Anything else is unexpected in code this new,
        and might leave the toolhead mid-move rather than back at
        center, so it's treated as unsafe to continue from and aborts
        the whole calibration (KASTUnsafeAbort) rather than silently
        testing further values from an unknown position. Logged via
        logging.exception either way so it's visible in klippy.log."""
        try:
            ok, err = self.safe_tester.test()
        except self.gcode.error as e:
            return False, None, str(e)
        except Exception as e:
            logging.exception("KAST: unexpected error during safe-zone test")
            raise KASTUnsafeAbort(
                "unexpected error during safe-zone test, stopping rather "
                "than continue from a possibly-unknown toolhead position: "
                "%s" % e)
        return ok, None, err

    def trial(self, sgt, current, homing_speed, samples):
        """Runs `samples` homing attempts at the given SGT/current/speed
        and returns a score dict. sgt=None leaves driver_SGT untouched
        (used by KAST_TEST to probe whatever is currently configured).
        Lower 'variance' and higher 'success_rate' is better; roughness
        (if available) penalizes the score."""
        if sgt is not None:
            self.driver.set_sgt(sgt)
        if current is not None:
            self.driver.set_current(current)
        test_fn = self._home_once
        if self.safe_tester is not None:
            test_fn = self._safe_zone_once
        else:
            # KASTSafeZoneTester uses its own explicit speed, not the
            # rail.homing_speed override this mechanism pokes.
            self._set_homing_speed(homing_speed)
        self.boop(sgt)

        successes = 0
        positions = []
        roughness_vals = []
        last_error = None
        for _ in range(samples):
            roughness, (ok, pos, err) = self.accel.measure(test_fn)
            if ok:
                successes += 1
                if pos is not None:
                    positions.append(pos)
                if roughness is not None:
                    roughness_vals.append(roughness)
            else:
                last_error = err

        success_rate = successes / float(samples)
        mean_pos = sum(positions) / len(positions) if positions else None
        pos_stddev = (statistics.pstdev(positions)
                      if len(positions) > 1 else 0.0)
        avg_roughness = (sum(roughness_vals) / len(roughness_vals)
                          if roughness_vals else None)

        # Composite score: reliability first, then repeatability, then
        # (if available) mechanical smoothness. Lower is better.
        score = (1.0 - success_rate) * 1000.0
        score += pos_stddev * 50.0
        if avg_roughness is not None:
            score += avg_roughness * 0.1

        return {
            'sgt': sgt,
            'current': current,
            'homing_speed': homing_speed,
            'success_rate': success_rate,
            'mean_pos': mean_pos,
            'pos_stddev': pos_stddev,
            'roughness': avg_roughness,
            'score': score,
            'last_error': last_error,
        }

    def sweep(self, sgt_min, sgt_max, sgt_step, current, homing_speed,
              samples):
        results = []
        sgt = sgt_min
        while sgt <= sgt_max:
            results.append(self.trial(sgt, current, homing_speed, samples))
            sgt += sgt_step
        return results

    def walk_from_baseline(self, baseline_sgt, sgt_min, sgt_max, sgt_step,
                            current, homing_speed, samples):
        """Tests baseline_sgt first (it should already be known-reliable),
        then steps one sgt_step at a time away from it in each
        direction the [sgt_min, sgt_max] range actually covers,
        stopping in that direction the moment a step isn't fully
        reliable. This bounds exploration to only as far as the
        printer actually keeps working, instead of committing to a
        fixed range regardless of what happens along the way.

        Note what this can and can't catch: it stops on a step that
        fails outright (samples come back short or erroring). It
        CANNOT detect a step that "succeeds" via a false/early
        StallGuard trigger, because Klipper reports the same
        configured position_endstop value for any triggered G28
        regardless of whether the trigger was real, there's nothing
        in that reported position for this method to compare. That
        failure mode is caught separately, in the homing macro itself
        (macros/kast.cfg's min_home_travel check), which measures
        actual physical movement before/after G28 rather than relying
        on Klipper's reported position, and aborts the whole
        calibration immediately if a move looks too short to be real.
        """
        results = []
        baseline = self.trial(baseline_sgt, current, homing_speed, samples)
        results.append(baseline)
        if baseline['success_rate'] < 1.0:
            return results

        for direction in (-1, 1):
            sgt = baseline_sgt + direction * sgt_step
            while sgt_min <= sgt <= sgt_max:
                r = self.trial(sgt, current, homing_speed, samples)
                results.append(r)
                if r['success_rate'] < 1.0:
                    break
                sgt += direction * sgt_step
        return results

    def search(self, sgt_min, sgt_max, sgt_step, currents, homing_speeds,
               samples, baseline_sgt=None):
        """Searches SGT for each candidate current/speed combination;
        returns the best overall result plus the full result table.
        Walks incrementally from baseline_sgt if given (see
        walk_from_baseline), otherwise sweeps the full range blindly
        (only happens when there's no known-working value to anchor
        to, e.g. a fresh printer)."""
        all_results = []
        try:
            for current in currents:
                for homing_speed in homing_speeds:
                    if baseline_sgt is not None:
                        all_results.extend(self.walk_from_baseline(
                            baseline_sgt, sgt_min, sgt_max, sgt_step,
                            current, homing_speed, samples))
                    else:
                        all_results.extend(
                            self.sweep(sgt_min, sgt_max, sgt_step, current,
                                       homing_speed, samples))
        finally:
            self.restore_homing_speed()
        # Prefer fully-reliable results, then best (lowest) score among
        # those, biased toward the middle of a reliable run so we don't
        # pick the very edge of the working range.
        reliable = [r for r in all_results if r['success_rate'] == 1.0]
        pool = reliable if reliable else all_results
        best = min(pool, key=lambda r: r['score'])
        if reliable:
            self.celebrate(best['sgt'], best['current'])
        return best, all_results


class KAST:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.fun_mode = config.getboolean('fun_mode', True)
        accel_chip = config.get('accel_chip', 'default')
        self.accel = KASTAccelHelper(self.printer, accel_chip)
        self.default_samples = config.getint('samples', 5, minval=1)
        self.default_sgt_step = config.getint('sgt_step', 8, minval=1)
        self.default_sgt_radius = config.getint('sgt_radius', 16, minval=1)
        # KAST_CALIBRATE tests near bed center rather than searching a
        # real endstop: a false StallGuard trigger there just stops
        # the test move early and gets logged, there's no hard stop
        # anywhere nearby to crash into. safe_zone_square is the total
        # span (e.g. 100 -> +/-50mm each direction from center).
        self.default_safe_zone_square = config.getfloat(
            'safe_zone_square', 100.0, above=0.0)
        self.default_safe_zone_speed = config.getfloat(
            'safe_zone_speed', 50.0, above=0.0)
        self.results_dir = os.path.expanduser(
            config.get('results_dir', '~/printer_data/config/kast_results'))
        self.enable_plots = config.getboolean('enable_plots', True)
        # scripts/kast_plot.py, two levels up from klippy/extras/kast.py
        # if the whole KAST repo was cloned/symlinked in (rather than
        # just this one file being copied in).
        default_plot_script = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', 'scripts', 'kast_plot.py'))
        self.plot_script = config.get('plot_script', default_plot_script)
        self.last_results = {}

        self.gcode.register_command(
            'KAST_CALIBRATE', self.cmd_KAST_CALIBRATE,
            desc=self.cmd_KAST_CALIBRATE_help)
        self.gcode.register_command(
            'KAST_TEST', self.cmd_KAST_TEST,
            desc=self.cmd_KAST_TEST_help)
        self.gcode.register_command(
            'KAST_STATUS', self.cmd_KAST_STATUS,
            desc=self.cmd_KAST_STATUS_help)
        self.gcode.register_command(
            'KAST_APPLY', self.cmd_KAST_APPLY,
            desc=self.cmd_KAST_APPLY_help)

    def _write_csv(self, stepper_name, all_results):
        """Dumps every trial from a calibration run to CSV, under
        results_dir/<stepper_name>/, for later plotting. Best-effort: a
        failure here shouldn't take down the calibration itself."""
        try:
            out_dir = os.path.join(self.results_dir, stepper_name)
            if not os.path.exists(out_dir):
                os.makedirs(out_dir)
            path = os.path.join(
                out_dir, "kast_%s_%d.csv" % (stepper_name, int(time.time())))
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'sgt', 'current', 'homing_speed', 'success_rate',
                    'pos_stddev', 'roughness', 'score'])
                for r in all_results:
                    writer.writerow([
                        r['sgt'],
                        '' if r['current'] is None else r['current'],
                        '' if r['homing_speed'] is None
                        else r['homing_speed'],
                        r['success_rate'], r['pos_stddev'],
                        '' if r['roughness'] is None else r['roughness'],
                        r['score']])
            return path
        except (IOError, OSError) as e:
            logging.warning("KAST: could not write results CSV: %s", e)
            return None

    def _plot_async(self, csv_path):
        """Fires scripts/kast_plot.py as a detached background process
        (mirrors klippain-shaketune's approach) so a slow/missing
        matplotlib on the host never blocks klippy's reactor. Returns
        the PNG path this will attempt to write, or None if plotting
        isn't available/enabled."""
        if not self.enable_plots:
            return None
        if not os.path.isfile(self.plot_script):
            logging.info(
                "KAST: plot script not found at %s (copy the whole "
                "KAST repo, not just kast.py, to enable auto-plots)",
                self.plot_script)
            return None
        png_path = os.path.splitext(csv_path)[0] + '.png'
        try:
            subprocess.Popen(
                [sys.executable, self.plot_script, csv_path,
                 '-o', png_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as e:
            logging.warning("KAST: could not launch plot script: %s", e)
            return None
        return png_path

    cmd_KAST_CALIBRATE_help = (
        "Search for a reliable driver_SGT near bed center (no real "
        "endstop search, so no crash risk) on a sensorless-homing stepper")

    def cmd_KAST_CALIBRATE(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        axis = gcmd.get('AXIS', stepper_name[-1] if stepper_name else 'x')
        sgt_step = gcmd.get_int('SGT_STEP', self.default_sgt_step, minval=1)
        samples = gcmd.get_int('SAMPLES', self.default_samples, minval=1)
        current_min = gcmd.get_float('CURRENT_MIN', None)
        current_max = gcmd.get_float('CURRENT_MAX', None)
        current_step = gcmd.get_float('CURRENT_STEP', 0.1, above=0.0)
        square_size = gcmd.get_float(
            'SQUARE_SIZE', self.default_safe_zone_square, above=0.0)
        safe_zone_speed = gcmd.get_float(
            'SAFE_ZONE_SPEED', self.default_safe_zone_speed, above=0.0)

        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        homed_axes = toolhead.get_status(eventtime).get('homed_axes', '')
        if axis.lower() not in homed_axes:
            raise gcmd.error(
                "KAST: '%s' isn't homed yet. KAST_CALIBRATE moves to bed "
                "center before testing, which needs a real position to "
                "move from. Run G28 first." % axis.upper())

        if self.fun_mode:
            gcmd.respond_info(
                "KAST warming up for '%s'... hold onto your belts!"
                % stepper_name)

        tuner = KASTStepperTuner(
            self, stepper_name, axis, config_section=None,
            safe_zone_travel=square_size / 2.0,
            safe_zone_speed=safe_zone_speed)

        # Default sweep range: only explores MORE sensitive (gentler)
        # settings than whatever's already configured, never less. A
        # known-working value means the printer already survives that
        # much force; there's no upside to defaulting into harsher
        # territory, only risk. For signed "sgt" drivers, lower is more
        # sensitive, so the default range stops AT the current value
        # rather than going past it. For unsigned "sg4_thrs" drivers,
        # higher is more sensitive, so it's the same idea in reverse.
        # Pass SGT_MIN/SGT_MAX explicitly to test the harsher direction
        # on purpose, this is only about what happens by default.
        range_min, range_max = tuner.driver.sgt_range
        current_sgt = tuner.driver.get_configured_sgt()
        if current_sgt is not None:
            radius = gcmd.get_int('SGT_RADIUS', self.default_sgt_radius,
                                   minval=1)
            if tuner.driver.sgt_signed:
                default_min = max(range_min, current_sgt - radius)
                default_max = current_sgt
            else:
                default_min = current_sgt
                default_max = min(range_max, current_sgt + radius)
            gcmd.respond_info(
                "KAST: found configured %s=%d on [%s], defaulting to "
                "SGT %d..%d" % (tuner.driver.config_key, current_sgt,
                                 tuner.driver.driver_name, default_min,
                                 default_max))
        else:
            default_min, default_max = range_min, range_max
            gcmd.respond_info(
                "KAST: no configured %s found on [%s], defaulting to "
                "the full range %d..%d"
                % (tuner.driver.config_key, tuner.driver.driver_name,
                   default_min, default_max))
        sgt_min = gcmd.get_int('SGT_MIN', default_min)
        sgt_max = gcmd.get_int('SGT_MAX', default_max)
        if sgt_min != default_min or sgt_max != default_max:
            gcmd.respond_info(
                "KAST: SGT_MIN/SGT_MAX given explicitly, overriding the "
                "default -> sweeping %d..%d" % (sgt_min, sgt_max))

        if current_min is not None and current_max is not None:
            currents = []
            c = current_min
            while c <= current_max + 1e-9:
                currents.append(round(c, 3))
                c += current_step
        else:
            # Without an explicit sweep, default to whatever home_current
            # a real homing move would actually use, not leave current
            # untouched at full run_current. StallGuard's threshold
            # depends heavily on current, testing at full current could
            # make everything look falsely "safe" and not reflect real
            # homing at all.
            home_current = None
            homing_state = self.printer.lookup_object(
                'gcode_macro _KAST_HOMING_STATE', None)
            if homing_state is not None:
                home_current = homing_state.variables.get('home_current')
            currents = [home_current]
            if home_current is not None:
                gcmd.respond_info(
                    "KAST: no CURRENT_MIN/MAX given, testing at "
                    "home_current=%.2fA (from _KAST_HOMING_STATE)"
                    % home_current)
            else:
                gcmd.respond_info(
                    "KAST: no CURRENT_MIN/MAX given and no "
                    "_KAST_HOMING_STATE macro found, testing at whatever "
                    "current is currently configured -- if that's full "
                    "run_current rather than a reduced homing current, "
                    "results may not reflect real homing behavior.")

        homing_speeds = [None]  # not swept; safe-zone testing uses its
                                 # own explicit SAFE_ZONE_SPEED instead
                                 # of the rail's homing_speed mechanism

        total_trials = (
            ((sgt_max - sgt_min) // sgt_step + 1)
            * len(currents) * len(homing_speeds) * samples)
        gcmd.respond_info(
            "KAST: testing SGT %d..%d for '%s' near bed center, +/-%.1fmm "
            "each direction (up to ~%d test cycles, fewer if it stops "
            "early on an unreliable step). This tests in free space, "
            "away from any hard stop, so a false trigger just logs and "
            "stops that move short, no crash risk, but keep an eye on "
            "it regardless."
            % (sgt_min, sgt_max, stepper_name, square_size / 2.0,
               total_trials))

        try:
            best, all_results = tuner.search(
                sgt_min, sgt_max, sgt_step, currents, homing_speeds,
                samples, baseline_sgt=current_sgt)
        except KASTUnsafeAbort as e:
            raise gcmd.error(
                "KAST_CALIBRATE stopped: %s\nNothing further was moved. "
                "Whatever was tested up to this point is not saved; "
                "consider a smaller SGT_STEP or investigating the "
                "endstop/wiring for that axis before retrying." % e)

        self.last_results[stepper_name] = {
            'best': best,
            'all': all_results,
            'config_key': tuner.driver.config_key,
            'driver_name': tuner.driver.driver_name,
        }

        csv_path = self._write_csv(stepper_name, all_results)
        if csv_path is not None:
            png_path = self._plot_async(csv_path)
            if png_path is not None:
                gcmd.respond_info(
                    "KAST: results saved to %s -- rendering graph to %s "
                    "in the background." % (csv_path, png_path))
            else:
                gcmd.respond_info(
                    "KAST: results saved to %s. Graph it with: "
                    "python3 scripts/kast_plot.py %s"
                    % (csv_path, csv_path))

        if best['success_rate'] < 1.0:
            msg = ("KAST: no fully false-trigger-free SGT/current found "
                   "for '%s'. Best attempt: SGT=%s CURRENT=%s "
                   "(success_rate=%.0f%%)"
                   % (stepper_name, best['sgt'], best['current'],
                      best['success_rate'] * 100))
            if best['last_error']:
                msg += " (%s)" % best['last_error']
            msg += ". Consider a smaller SGT_RADIUS or checking mechanics."
            gcmd.respond_info(msg)
        else:
            msg = ("KAST: best result for '%s' -> SGT=%s"
                   % (stepper_name, best['sgt']))
            if best['current'] is not None:
                msg += " CURRENT=%.2fA" % best['current']
            if best['roughness'] is not None:
                msg += " (roughness=%.3f)" % best['roughness']
            msg += (". This value never false-triggered near bed center, "
                     "which doesn't by itself confirm it's sensitive "
                     "enough to trigger reliably at the real endstop -- "
                     "run KAST_TEST STEPPER=%s to confirm that with real "
                     "homing before KAST_APPLY." % stepper_name)
            gcmd.respond_info(msg)

    cmd_KAST_TEST_help = (
        "Repeatedly home a stepper at its CURRENT driver_SGT/current/"
        "speed (no sweeping, nothing is changed) to sanity-check "
        "reliability")

    def cmd_KAST_TEST(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        axis = gcmd.get('AXIS', stepper_name[-1] if stepper_name else 'x')
        samples = gcmd.get_int('SAMPLES', self.default_samples, minval=1)

        tuner = KASTStepperTuner(self, stepper_name, axis, config_section=None)
        gcmd.respond_info(
            "KAST: testing '%s' as currently configured -- %d homing "
            "attempts, nothing will be changed." % (stepper_name, samples))

        result = tuner.trial(None, None, None, samples)

        msg = ("KAST: '%s' success_rate=%.0f%% pos_stddev=%.4f"
               % (stepper_name, result['success_rate'] * 100,
                  result['pos_stddev']))
        if result['roughness'] is not None:
            msg += " roughness=%.3f" % result['roughness']
        if result['success_rate'] < 1.0 and result['last_error']:
            msg += " last_error=%r" % result['last_error']
        gcmd.respond_info(msg)
        if result['success_rate'] < 1.0:
            gcmd.respond_info(
                "KAST: current settings are NOT fully reliable for '%s'. "
                "Consider running KAST_CALIBRATE." % stepper_name)

    cmd_KAST_STATUS_help = "Show the last KAST_CALIBRATE result(s)"

    def cmd_KAST_STATUS(self, gcmd):
        if not self.last_results:
            gcmd.respond_info("KAST: no calibration has been run yet.")
            return
        for stepper_name, data in self.last_results.items():
            best = data['best']
            gcmd.respond_info(
                "%s: SGT=%s CURRENT=%s success_rate=%.0f%% score=%.2f"
                % (stepper_name, best['sgt'], best['current'],
                   best['success_rate'] * 100, best['score']))

    cmd_KAST_APPLY_help = (
        "Persist the last KAST_CALIBRATE result for a stepper into the "
        "saved config (requires SAVE_CONFIG afterwards)")

    def cmd_KAST_APPLY(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        data = self.last_results.get(stepper_name)
        if data is None:
            raise gcmd.error(
                "KAST: no calibration result for '%s'; run "
                "KAST_CALIBRATE first." % stepper_name)
        best = data['best']
        configfile = self.printer.lookup_object('configfile')
        applied = ["%s=%s" % (data['config_key'], best['sgt'])]
        # driver_SGT / driver_SGTHRS live under the driver's own config
        # section (e.g. "tmc2240 stepper_x"), not the stepper section.
        configfile.set(data['driver_name'], data['config_key'],
                        str(best['sgt']))
        if best['homing_speed'] is not None:
            configfile.set(stepper_name, 'homing_speed',
                            "%.3f" % best['homing_speed'])
            applied.append("homing_speed=%.3f" % best['homing_speed'])
        if best['current'] is not None:
            if self.printer.lookup_object(
                    'gcode_macro _KAST_HOMING_STATE', None) is not None:
                configfile.set('gcode_macro _KAST_HOMING_STATE',
                                'variable_home_current',
                                "%.3f" % best['current'])
                applied.append("home_current=%.3f" % best['current'])
            else:
                gcmd.respond_info(
                    "KAST: best homing current was %.3fA, but there's no "
                    "'home_current' config field in Klipper and no "
                    "_KAST_HOMING_STATE macro found to persist it into -- "
                    "apply it by hand in whatever sets your homing "
                    "current." % best['current'])
        gcmd.respond_info(
            "KAST: staged %s for [%s]. Run SAVE_CONFIG to write it out "
            "and restart." % (", ".join(applied), stepper_name))


def load_config(config):
    return KAST(config)
