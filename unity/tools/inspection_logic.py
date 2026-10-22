"""Pure demo alarm calculations. ppm limits are scenario settings, not occupational limits."""
import math


def accumulate_exposure(previous_time, now, previous_ppm, ppm, exposure):
    if previous_time is None or now < previous_time:
        return 0.0 if previous_time is not None else exposure
    # Missing samples do not imply zero exposure, or justify estimating an unobserved interval.
    dt = now-previous_time
    if dt > 1.0:
        return exposure
    return exposure + .5*(max(0., previous_ppm)+max(0., ppm))*dt


def alarm_gas(ppm, exposure, config):
    return math.isfinite(ppm) and (ppm >= config['gas_alarm_ppm'] or exposure >= config['exposure_alarm_ppm_s'])
