"""Random generators on every numpy: Generator from 1.17 (Ubuntu 20.04), RandomState before."""

from __future__ import annotations

import numpy as np


class LegacyGenerator:
    """The numpy.random.Generator methods the stack uses, on top of RandomState (numpy < 1.17)."""

    def __init__(self, seed=None):
        self._state = np.random.RandomState(seed)

    def random(self, size=None):
        """Return floats in [0, 1)."""
        return self._state.random_sample(size)

    def integers(self, low, high=None, size=None, dtype=np.int64, endpoint: bool = False):
        """Return integers of dtype in [low, high) ([low, high] with endpoint)."""
        if high is None:
            low, high = 0, low
        return self._state.randint(low, high + 1 if endpoint else high, size, dtype)

    def normal(self, loc=0.0, scale=1.0, size=None):
        """Return normal samples."""
        return self._state.normal(loc, scale, size)

    def standard_normal(self, size=None):
        """Return standard normal samples."""
        return self._state.standard_normal(size)

    def uniform(self, low=0.0, high=1.0, size=None):
        """Return uniform samples in [low, high)."""
        return self._state.uniform(low, high, size)

    def choice(self, a, size=None, replace: bool = True, p=None):
        """Return a random sample of a."""
        return self._state.choice(a, size, replace, p)

    def permutation(self, x):
        """Return a shuffled copy (or a permuted range)."""
        return self._state.permutation(x)

    def shuffle(self, x):
        """Shuffle x in place."""
        self._state.shuffle(x)

    def __getattr__(self, name: str):
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._state, name)  # other distributions share their names


def default_rng(seed=None):
    """Return numpy.random.default_rng(seed), or a LegacyGenerator on numpy < 1.17."""
    make = getattr(np.random, 'default_rng', None)
    return make(seed) if make is not None else LegacyGenerator(seed)


def default_rngs(seed=None, n: int = 1) -> list:
    """Return n independent generators derived from seed (one per worker, robot or trial)."""
    sequence = getattr(np.random, 'SeedSequence', None)  # numpy 1.17+
    if sequence is not None:
        root = seed if isinstance(seed, sequence) else sequence(seed)
        return [np.random.default_rng(child) for child in root.spawn(n)]  # py38: ok
    if seed is None:
        return [LegacyGenerator() for _ in range(n)]
    words = _seed_words(seed)
    return [LegacyGenerator([*words, i]) for i in range(n)]


def _seed_words(seed) -> list:
    """Return seed (a non-negative int or a sequence of them) as 32-bit words."""
    values = [seed] if np.isscalar(seed) else list(seed)
    words = []
    for value in values:
        rest = int(value)
        if rest < 0:
            raise ValueError(f'seed must be non-negative, got {rest}')
        words.append(rest & 0xFFFFFFFF)
        while rest > 0xFFFFFFFF:
            rest >>= 32
            words.append(rest & 0xFFFFFFFF)
    return words
