"""NeuPAN's DUNE model of the robot's footprint: its vertices, its record and its training."""

# DUNE maps an obstacle point (robot frame) to the dual variables of the point's distance to the
# footprint. It depends on the footprint only, never on the kinematics: train it once per
# footprint, keep the checkpoint with the record of what it was trained for (<model>.json), and
# the controller refuses a checkpoint whose footprint is not the robot's.

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import os
import shutil
import tempfile

import numpy as np

from x30_utils.io.files import read_json, write_json_atomic

VERTEX_TOLERANCE = 1e-3
"""Vertices further apart than this (m): another footprint."""


def footprint_vertices(footprint) -> np.ndarray:
    """Return the footprint's corners as NeuPAN vertices, (2, 4) counter-clockwise."""
    return np.asarray(footprint.corners(), dtype=float).T


def record_path(checkpoint: str) -> str:
    """Return the record of a checkpoint: model.pth -> model.json."""
    return os.path.splitext(checkpoint)[0] + '.json'


@dataclass(frozen=True)
class DuneRecord:
    """What a checkpoint was trained for and how well it learned it."""

    vertices: list = field(default_factory=list)
    data_range: list = field(default_factory=list)
    data_size: int = 0
    epochs: int = 0
    seed: int = 0
    mean_error_m: float = -1.0  # -1: not measured
    max_error_m: float = -1.0
    max_over_m: float = -1.0  # the largest overestimate: a point seen further than it is

    @classmethod
    def load(cls, checkpoint: str) -> DuneRecord:
        """Read the record next to a checkpoint (OSError, ValueError when unusable)."""
        path = record_path(checkpoint)
        data, error = read_json(path)
        if error:
            raise ValueError(error)
        if data is None:
            raise OSError(f'{path} not found')
        if not isinstance(data, dict) or 'vertices' not in data:
            raise ValueError(f'{path}: no vertices')
        known = {name: data[name] for name in cls.__dataclass_fields__ if name in data}
        return cls(**known)

    def save(self, checkpoint: str):
        """Write the record next to a checkpoint."""
        write_json_atomic(record_path(checkpoint), asdict(self), backup=False)

    @property
    def range_m(self) -> float:
        """Distance from the robot within which the model was trained (m)."""
        if len(self.data_range) != 4:
            return 0.0
        return float(min(abs(value) for value in self.data_range))


def check_checkpoint(checkpoint: str, vertices: np.ndarray) -> list[str]:
    """Return why a checkpoint cannot serve these vertices (empty: it can)."""
    if not checkpoint:
        return ['no DUNE checkpoint set (neupan/dune_checkpoint)']
    if not os.path.isfile(checkpoint):
        return [f'DUNE checkpoint {checkpoint} not found: train one (x30_navigation README)']
    try:
        record = DuneRecord.load(checkpoint)
    except (OSError, ValueError) as error:
        return [f'DUNE checkpoint without a usable record ({error}): train it again']
    trained = np.asarray(record.vertices, dtype=float)
    wanted = np.asarray(vertices, dtype=float)
    if trained.shape != wanted.shape or not np.allclose(trained, wanted, atol=VERTEX_TOLERANCE):
        return [
            f'DUNE checkpoint {checkpoint} was trained for another footprint '
            f'({trained.tolist()} != {wanted.tolist()}): train it for the robot profile'
        ]
    return []


def polygon_distance(vertices: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Return the distance of (N, 2) points to a convex polygon (2, M); 0 inside."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    corners = np.asarray(vertices, dtype=float).T
    edges = np.roll(corners, -1, axis=0) - corners
    rel = points[:, None, :] - corners[None, :, :]
    t = np.clip((rel * edges).sum(axis=2) / (edges * edges).sum(axis=1), 0.0, 1.0)
    nearest = corners[None] + t[:, :, None] * edges[None]
    distance = np.linalg.norm(points[:, None, :] - nearest, axis=2).min(axis=1)
    cross = edges[None, :, 0] * rel[:, :, 1] - edges[None, :, 1] * rel[:, :, 0]
    inside = (cross >= 0.0).all(axis=1)
    return np.where(inside, 0.0, distance)


def model_distance(model, vertices: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Return the distances DUNE gives for (N, 2) robot-frame points (needs torch, neupan)."""
    from neupan.util import gen_inequal_from_vertex
    import torch

    g, h = gen_inequal_from_vertex(np.asarray(vertices, dtype=float))
    p = torch.as_tensor(np.asarray(points, dtype=np.float32).reshape(-1, 2))
    with torch.no_grad():
        mu = model(p)  # (N, edges)
    gap = p @ torch.as_tensor(g, dtype=torch.float32).T - torch.as_tensor(h.T, dtype=torch.float32)
    return (mu * gap).sum(dim=1).numpy()


def load_model(checkpoint: str, edges: int):
    """Return the DUNE network of a checkpoint, for evaluation (needs torch, neupan)."""
    from neupan.blocks.obs_point_net import ObsPointNet
    import torch

    model = ObsPointNet(2, edges)
    model.load_state_dict(torch.load(checkpoint, map_location='cpu', weights_only=True))
    model.eval()
    return model


def measure(model, vertices: np.ndarray, range_m: float, seed: int) -> dict:
    """Return the record's error fields: DUNE's distances against the exact ones (m)."""
    points = validation_points(range_m, 20000, seed)
    signed = model_distance(model, vertices, points) - polygon_distance(vertices, points)
    return {
        'mean_error_m': round(float(np.abs(signed).mean()), 5),
        'max_error_m': round(float(np.abs(signed).max()), 5),
        'max_over_m': round(max(0.0, float(signed.max())), 5),
    }


def remeasure(checkpoint: str) -> DuneRecord:
    """Measure a checkpoint again and rewrite its record's error fields."""
    record = DuneRecord.load(checkpoint)
    vertices = np.asarray(record.vertices, dtype=float)
    model = load_model(checkpoint, vertices.shape[1])
    updated = replace(record, **measure(model, vertices, record.range_m, record.seed))
    updated.save(checkpoint)
    return updated


def validation_points(range_m: float, count: int, seed: int) -> np.ndarray:
    """Return points within range_m of the robot, half of them within 2 m (where it counts)."""
    rng = np.random.default_rng(seed + 1)
    near = rng.uniform(-min(2.0, range_m), min(2.0, range_m), (count // 2, 2))
    far = rng.uniform(-range_m, range_m, (count - count // 2, 2))
    return np.vstack((near, far))


def train(
    vertices: np.ndarray,
    checkpoint: str,
    data_size: int = 100000,
    data_range: tuple = (-8.0, -8.0, 8.0, 8.0),
    epochs: int = 1500,
    seed: int = 0,
    threads: int = 0,
    **train_kwargs,
) -> DuneRecord:
    """Train DUNE for the vertices with NeuPAN's trainer; write checkpoint and its record."""
    from neupan.blocks.dune_train import DUNETrain
    from neupan.blocks.obs_point_net import ObsPointNet
    from neupan.configuration import np_to_tensor
    from neupan.util import gen_inequal_from_vertex
    import torch

    np.random.seed(seed)  # NeuPAN draws the data set and the rotations with np.random  # noqa: NPY002
    torch.manual_seed(seed)
    if threads > 0:
        torch.set_num_threads(threads)
    vertices = np.asarray(vertices, dtype=float)
    g, h = gen_inequal_from_vertex(vertices)
    if g is None:
        raise ValueError('the footprint is not a convex polygon')
    model = ObsPointNet(2, g.shape[0])
    work = tempfile.mkdtemp(prefix='dune_')
    try:
        trainer = DUNETrain(model, np_to_tensor(g), np_to_tensor(h), work)
        # the learning rate halves every 30 % of the epochs (NeuPAN's 1500 of 5000)
        train_kwargs.setdefault('decay_freq', max(1, round(epochs * 0.3)))
        best = trainer.start(
            data_size=data_size,
            data_range=list(data_range),
            epoch=epochs,
            save_freq=epochs,
            **train_kwargs,
        )
        model.load_state_dict(torch.load(best, map_location='cpu', weights_only=True))
        os.makedirs(os.path.dirname(os.path.abspath(checkpoint)), exist_ok=True)
        shutil.copyfile(best, checkpoint)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    model.eval()
    range_m = float(min(abs(value) for value in data_range))
    record = DuneRecord(
        vertices=vertices.round(6).tolist(),
        data_range=[float(value) for value in data_range],
        data_size=int(data_size),
        epochs=int(epochs),
        seed=int(seed),
        **measure(model, vertices, range_m, seed),
    )
    record.save(checkpoint)
    return record
