"""train_dune: train NeuPAN's DUNE model for the footprint of a robot profile (no ROS).

    rosrun x30_navigation train_dune --robot-params $(rospack find x30_bringup)/config/robot.yaml \
        --out $(rospack find x30_navigation)/models/dune_x30.pth

The checkpoint is written with its record (dune_x30.json): the footprint, the training and the
distance error measured on points around the robot. Needs NeuPAN (tools/install_neupan.sh).
"""

from __future__ import annotations

import argparse
import sys

import yaml

from x30_navigation.core.dune_model import footprint_vertices, train
from x30_utils.robot import Footprint


def read_footprint(path: str) -> Footprint:
    """Return the footprint of a robot profile file (x30_bringup/config/robot.yaml)."""
    with open(path, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream) or {}
    values = profile.get('footprint') or {}
    names = ('front', 'rear', 'half_width')
    known = {name: float(values[name]) for name in names if name in values}
    return Footprint(**known)


def parse(argv: list[str]) -> argparse.Namespace:
    """Return the command line options."""
    parser = argparse.ArgumentParser(prog='train_dune', description=__doc__.splitlines()[0])
    parser.add_argument('--robot-params', required=True, help='robot profile YAML (footprint)')
    parser.add_argument('--out', required=True, help='checkpoint to write (.pth)')
    parser.add_argument(
        '--epochs', type=int, default=1500, help='the learning rate halves every 30 %%'
    )
    parser.add_argument('--data-size', type=int, default=100000, help='points in the data set')
    parser.add_argument(
        '--range', type=float, default=8.0, help='points up to this far from base_link (m)'
    )
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--threads', type=int, default=0, help='torch threads (0: all)')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Train and report the record; 0 on success."""
    args = parse(sys.argv[1:] if argv is None else argv)
    footprint = read_footprint(args.robot_params)
    vertices = footprint_vertices(footprint)
    print(f'footprint {vertices.T.tolist()} (m, base_link), {args.epochs} epochs')
    span = (-args.range, -args.range, args.range, args.range)
    record = train(
        vertices,
        args.out,
        data_size=args.data_size,
        data_range=span,
        epochs=args.epochs,
        seed=args.seed,
        threads=args.threads,
    )
    print(
        f'{args.out}: distance error mean {record.mean_error_m:.4f} m, '
        f'max {record.max_error_m:.4f} m within {record.range_m:.1f} m'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
