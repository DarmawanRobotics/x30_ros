"""Path controllers, registered in CONTROLLERS (entry point group 'x30.controllers')."""

# neupan needs NeuPAN (an optional dependency), imported only when that controller is built.

from __future__ import annotations

from x30_navigation.core.controllers.base import (
    CONTROLLERS,
    Bias,
    Command,
    ControllerLimits,
    PathController,
    TrackRequest,
)
from x30_navigation.core.controllers.holonomic import HolonomicController
from x30_navigation.core.controllers.lqr import LqrController
from x30_navigation.core.controllers.mppi import MppiController
from x30_navigation.core.controllers.neupan_mpc import NeupanController
from x30_navigation.core.controllers.pid import PidController
from x30_navigation.core.controllers.pure_pursuit import PurePursuitController
from x30_navigation.core.controllers.stanley import StanleyController

__all__ = [
    'CONTROLLERS',
    'Bias',
    'Command',
    'ControllerLimits',
    'HolonomicController',
    'LqrController',
    'MppiController',
    'NeupanController',
    'PathController',
    'PidController',
    'PurePursuitController',
    'StanleyController',
    'TrackRequest',
]
