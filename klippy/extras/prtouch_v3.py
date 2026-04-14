# prtouch support
#
# Copyright (C) 2018-9999  Creality <wangyulong878@sina.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


# PROBE PROBE_SPEED=5 LIFT_SPEED=5 SAMPLES=5 SAMPLE_RETRACT_DIST=3 SAMPLES_RESULT='median'
# SET_KINEMATIC_POSITION X=359 Y=-10

from . import prtouch_v3_wrapper
from . import probe as probes

def load_config(config):
    prtouch = prtouch_v3_wrapper.PRTouchEndstopWrapper(config)
    config.get_printer().add_object('axis_twist_compensation', prtouch)
    config.get_printer().add_object('probe', probes.PrinterProbe(config, prtouch))
    return prtouch
