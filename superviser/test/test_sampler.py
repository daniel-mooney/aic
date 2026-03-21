import pytest
import numpy as np

from superviser.planner.sampler import UniformEuclieanSampler

def test_sampler() -> None:
    low = np.array([0.0, 1.0, 2.0])
    high=np.array([3.0, 2.0, 3.0])
    
    sampler = UniformEuclieanSampler(low, high)

    for _ in range(1000):
        x = sampler.sample()

        assert (x >= low).all()
        assert (x <= high).all()
