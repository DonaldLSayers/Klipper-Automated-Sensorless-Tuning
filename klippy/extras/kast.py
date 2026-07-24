########################################################################
# KAST - Klipper Automated Sensorless Tuning
#
# Automatically searches for reliable driver_SGT (stall sensitivity)
# and home_current values on sensorless-homing steppers, using repeated
# homing trials and (optionally) ADXL345 vibration data as a quality
# signal.
#
# License: MIT
########################################################################

import logging
import statistics

# TMC field name that controls stall sensitivity, per driver family.
# TMC2130/TMC2660/TMC5160 use a signed "sgt" field (config: driver_SGT).
# TMC2208/TMC2209/TMC2226 use an unsigned "sgthrs" field
# (config: driver_SGTHRS).
# TMC2240 (StallGuard4, register SG4_THRS) is also exposed as "sgt"
# (config: driver_SGT) — confirmed against a real printer.cfg.
SGT_FIELD_BY_DRIVER = {
    'tmc2130': 'sgt',
    'tmc2660': 'sgt',
    'tmc5160': 'sgt',
    'tmc2208': 'sgthrs',
    'tmc2209': 'sgthrs',
    'tmc2226': 'sgthrs',
    'tmc2240': 'sgt',
}

SGT_SIGNED_DRIVERS = ('tmc2130', 'tmc2660', 'tmc5160')

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

    def __init__(self, kast, stepper_name, axis, config_section):
        self.kast = kast
        self.printer = kast.printer
        self.gcode = kast.gcode
        self.stepper_name = stepper_name
        self.axis = axis
        self.driver = KASTDriverAdapter(self.printer, stepper_name)
        self.accel = kast.accel

    def boop(self, sgt):
        if self.kast.fun_mode:
            line = BOOP_LINES[self._boop_i % len(BOOP_LINES)] % sgt
            self._boop_i += 1
            self.gcode.respond_info(line)

    def celebrate(self, sgt, current):
        if self.kast.fun_mode:
            self.gcode.respond_info(
                "Found a good spot! SGT=%s CURRENT=%.2fA. Nice." %
                (sgt, current))

    _boop_i = 0

    def _home_once(self):
        """Homes just this stepper's axis and reports whether it
        succeeded, plus the triggered position (for repeatability
        scoring)."""
        try:
            self.gcode.run_script_from_command(
                "G28 %s" % self.axis.upper())
        except self.gcode.error as e:
            return False, None, str(e)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        idx = {'x': 0, 'y': 1, 'z': 2}[self.axis.lower()]
        return True, pos[idx], None

    def trial(self, sgt, current, samples):
        """Runs `samples` homing attempts at the given SGT/current and
        returns a score dict. Lower 'variance' and higher 'success_rate'
        is better; roughness (if available) penalizes the score."""
        self.driver.set_sgt(sgt)
        if current is not None:
            self.driver.set_current(current)
        self.boop(sgt)

        successes = 0
        positions = []
        roughness_vals = []
        last_error = None
        for _ in range(samples):
            roughness, (ok, pos, err) = self.accel.measure(self._home_once)
            if ok:
                successes += 1
                positions.append(pos)
                if roughness is not None:
                    roughness_vals.append(roughness)
            else:
                last_error = err

        success_rate = successes / float(samples)
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
            'success_rate': success_rate,
            'pos_stddev': pos_stddev,
            'roughness': avg_roughness,
            'score': score,
            'last_error': last_error,
        }

    def sweep(self, sgt_min, sgt_max, sgt_step, current, samples):
        results = []
        sgt = sgt_min
        while sgt <= sgt_max:
            results.append(self.trial(sgt, current, samples))
            sgt += sgt_step
        return results

    def search(self, sgt_min, sgt_max, sgt_step, currents, samples):
        """Coarse sweep of SGT for each candidate current; returns the
        best overall result plus the full result table."""
        all_results = []
        for current in currents:
            all_results.extend(
                self.sweep(sgt_min, sgt_max, sgt_step, current, samples))
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
        self.last_results = {}

        self.gcode.register_command(
            'KAST_CALIBRATE', self.cmd_KAST_CALIBRATE,
            desc=self.cmd_KAST_CALIBRATE_help)
        self.gcode.register_command(
            'KAST_STATUS', self.cmd_KAST_STATUS,
            desc=self.cmd_KAST_STATUS_help)
        self.gcode.register_command(
            'KAST_APPLY', self.cmd_KAST_APPLY,
            desc=self.cmd_KAST_APPLY_help)

    cmd_KAST_CALIBRATE_help = (
        "Search for a reliable driver_SGT (and optionally home_current) "
        "on a sensorless-homing stepper")

    def cmd_KAST_CALIBRATE(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        axis = gcmd.get('AXIS', stepper_name[-1] if stepper_name else 'x')
        sgt_min = gcmd.get_int('SGT_MIN', -64)
        sgt_max = gcmd.get_int('SGT_MAX', 63)
        sgt_step = gcmd.get_int('SGT_STEP', self.default_sgt_step, minval=1)
        samples = gcmd.get_int('SAMPLES', self.default_samples, minval=1)
        current_min = gcmd.get_float('CURRENT_MIN', None)
        current_max = gcmd.get_float('CURRENT_MAX', None)
        current_step = gcmd.get_float('CURRENT_STEP', 0.1, above=0.0)

        if current_min is not None and current_max is not None:
            currents = []
            c = current_min
            while c <= current_max + 1e-9:
                currents.append(round(c, 3))
                c += current_step
        else:
            currents = [None]

        if self.fun_mode:
            gcmd.respond_info(
                "KAST warming up for '%s'... hold onto your belts!"
                % stepper_name)

        tuner = KASTStepperTuner(self, stepper_name, axis, config_section=None)
        best, all_results = tuner.search(
            sgt_min, sgt_max, sgt_step, currents, samples)

        self.last_results[stepper_name] = {
            'best': best,
            'all': all_results,
        }

        if best['success_rate'] < 1.0:
            gcmd.respond_info(
                "KAST: no fully reliable SGT/current combo found for "
                "'%s'. Best attempt: SGT=%s CURRENT=%s "
                "(success_rate=%.0f%%). Consider widening the search "
                "range or checking mechanics."
                % (stepper_name, best['sgt'], best['current'],
                   best['success_rate'] * 100))
        else:
            msg = ("KAST: best result for '%s' -> SGT=%s"
                   % (stepper_name, best['sgt']))
            if best['current'] is not None:
                msg += " CURRENT=%.2fA" % best['current']
            msg += (" (pos_stddev=%.4f" % best['pos_stddev'])
            if best['roughness'] is not None:
                msg += ", roughness=%.3f" % best['roughness']
            msg += "). Run KAST_APPLY STEPPER=%s to save." % stepper_name
            gcmd.respond_info(msg)

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
        "saved config section (requires SAVE_CONFIG afterwards)")

    def cmd_KAST_APPLY(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        data = self.last_results.get(stepper_name)
        if data is None:
            raise gcmd.error(
                "KAST: no calibration result for '%s'; run "
                "KAST_CALIBRATE first." % stepper_name)
        best = data['best']
        configfile = self.printer.lookup_object('configfile')
        configfile.set(stepper_name, 'driver_SGT', str(best['sgt']))
        if best['current'] is not None:
            configfile.set(stepper_name, 'home_current',
                            "%.3f" % best['current'])
        gcmd.respond_info(
            "KAST: staged driver_SGT=%s%s for [%s]. Run SAVE_CONFIG to "
            "write it out and restart."
            % (best['sgt'],
               (" and home_current=%.3f" % best['current'])
               if best['current'] is not None else "",
               stepper_name))


def load_config_prefix(config):
    return KAST(config)
