"""Read-only connection discovery for the fixed legacy or wireless-calibration app.

The calibration branch verifies identity and live DWR v2 only. It never grants
motion, certifies physical torque-off, writes the board, or invokes an updater.
"""
import discover_unoq as base


def discover(*args, **kwargs):
    kwargs['allow_calibration'] = True
    return base.discover(*args, **kwargs)


def main():
    return base.main(allow_calibration=True)


if __name__ == '__main__':
    raise SystemExit(main())
