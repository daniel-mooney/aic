from abc import ABC, abstractmethod
from planner.types import State

import numpy as np

class StateSampler(ABC):
    """Samples from a rectangular configuration space"""

    def __init__(self, low: State, high: State, dim: tuple[int, ...]) -> None:
        self._low = low
        self._high = high
        self._dim = dim

    @abstractmethod
    def sample(self) -> State:
        pass

class UniformEuclieanSampler(StateSampler):
    """Samples uniformly from configuration space."""

    def __init__(self, low: State, high: State, dim: tuple[int, ...]) -> None:
        super().__init__(low, high, dim)

    def sample(self) -> State:
        return np.random.uniform(self._low, self._high, size=self._dim)
