import pytest
import numpy as np

from superviser.planner.collision_checker import RectangularObstacle2d, RectangularObstacle3d, CollisionChecker

@pytest.fixture
def rectangle() -> RectangularObstacle3d:
    return RectangularObstacle3d(
        x_span=[0.0, 3.0],
        y_span=[1.0, 4.0],
        z_span=[-1.0, 4.0],
    )

@pytest.fixture
def rectangle2d() -> RectangularObstacle2d:
    return RectangularObstacle2d(
        x_span=[-1.0, 1.0],
        y_span=[-2.0, 1.0],
    )

def test_rectangle3d_contained_point(rectangle: RectangularObstacle3d) -> None:
    a = np.array([1.0, 2.0, 0.0])
    assert rectangle.contains(a)

def test_rectangle3d_outside_point(rectangle: RectangularObstacle3d) -> None:
    a = np.array([-1.0, 2.0, 3.0])
    b = np.array([2.0, 0.0, 1.0])
    c = np.array([1.0, 2.0, 5.0])

    assert not rectangle.contains(a)
    assert not rectangle.contains(b)
    assert not rectangle.contains(c)

def test_rectangle3d_contained_line(rectangle: RectangularObstacle3d) -> None:
    a = np.array([-1.0, 0.0, 0.0])
    b = np.array([5.0, 3.0, 2.0])

    assert rectangle.contains_line(a, b)

def test_rectangle3d_outside_line(rectangle: RectangularObstacle3d) -> None:
    a = np.array([-1.0, 2.0, 10.0])
    b = np.array([4.0, 2.0, 15.0])

    assert not rectangle.contains_line(a, b)

def test_rectangle3d_contained_parallel_line(rectangle: RectangularObstacle3d) -> None:
    a = np.array([1.0, 2.0, -3.0])
    b = np.array([1.0, 2.0, 7.0])

    assert rectangle.contains_line(a, b)

def test_rectangle2d_contained_point(rectangle2d: RectangularObstacle2d) -> None:
    a = np.array([0.0, 0.0])
    b = np.array([0.8, -1.2])

    assert rectangle2d.contains(a)
    assert rectangle2d.contains(b)

def test_rectangle2d_outside_point(rectangle2d: RectangularObstacle2d) -> None:
    a = np.array([3.0, 0.0])
    b = np.array([0.8, -2.2])

    assert not rectangle2d.contains(a)
    assert not rectangle2d.contains(b)

def test_rectangle2d_contains_line(rectangle2d: RectangularObstacle2d) -> None:
    a = np.array([-2.0, 0])
    b = np.array([4.0, -1.0])

    c = np.array([0.0, 0.0])

    assert rectangle2d.contains_line(a, b)
    assert rectangle2d.contains_line(a, c)

def test_rectangle2d_outside_line(rectangle2d: RectangularObstacle2d) -> None:
    a = np.array([-0.5, 5.0])
    b = np.array([2.0, 3.0])

    assert not rectangle2d.contains_line(a, b)
