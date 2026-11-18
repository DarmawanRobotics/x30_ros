"""inspection/add_target: a target from the PTZ camera's view where the robot stands."""

from __future__ import annotations

from x30_inspection.core import plan as plans
from x30_utils.io import valid_name


def clamp(value: float, bounds: tuple[float, float]) -> float:
    """Return value inside bounds."""
    return max(bounds[0], min(bounds[1], value))


def target_of(request, pan: float, tilt: float, zoom: float) -> plans.Target:
    """Return the Target of an AddTarget request at the camera's current view."""
    if not valid_name(request.name):
        raise plans.PlanError(f'target name {request.name!r}: use 1-48 of a-z, 0-9, _ and -')
    if not (request.thermal or request.acoustic):
        raise plans.PlanError('a target checks thermal, acoustic or both')
    if request.max_temp_c < 0.0 or request.strips > 16:
        raise plans.PlanError('max_temp_c >= 0 and strips <= 16')
    return plans.Target(
        name=request.name,
        pan_deg=round(clamp(pan, plans.PAN), 2),
        tilt_deg=round(clamp(tilt, plans.TILT), 2),
        zoom=round(clamp(zoom, plans.ZOOM), 1),
        thermal=plans.ThermalTarget(
            enabled=bool(request.thermal),
            strips=int(request.strips),
            max_temp_c=float(request.max_temp_c),
        ),
        acoustic=plans.AcousticTarget(enabled=bool(request.acoustic)),
    )


def add_target_from_camera(node, request) -> tuple[bool, str, str, int]:
    """Teach a target; return (ok, message, point, targets of the point)."""
    if not node.site:
        return False, 'no site loaded (site/load)', '', 0
    if node.busy.locked():
        return False, 'an inspection is running: the camera is not where it was aimed', '', 0
    here = node.robot_xy()
    if here is None:
        return False, 'robot pose unknown (not localized?)', '', 0
    view = node.camera.pose()
    if view is None:
        return False, 'no position from the camera (hikmicro_driver running?)', '', 0
    try:
        target = target_of(request, *view)
        path = plans.plan_path(node.config.sites_dir, node.site)
        grown, point = plans.add_target(plans.load(path), request.point, here[0], here[1],
                                        node.config.match_radius_m, target)  # fmt: skip
        plans.save(path, grown)
    except (plans.PlanError, OSError) as error:
        return False, str(error), '', 0
    message = (
        f'{target.name} at {point.name} ({point.x:.2f}, {point.y:.2f}): pan {target.pan_deg}, '
        f'tilt {target.tilt_deg}, zoom {target.zoom}'
    )
    return True, message, point.name, len(point.targets)
