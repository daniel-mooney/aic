import pytest
import numpy as np

from superviser.planner.indexer import KDTree

@pytest.fixture
def tree() -> KDTree[float, None]:
    return KDTree(
        dim=2,
        distance_metric=lambda a,b: np.linalg.norm(a - b, ord=2),
        axis_dist_metric= lambda a,b, _: abs(a - b),
    )

def test_knearest_single_point(tree: KDTree[float, None]) -> None:
    point = np.array([1.0, 2.0])
    tree.add(point)

    query = np.array([1.0, 0.0])
    result = tree.knearest(query, k=1)

    assert len(result) == 1
    dist, key, _ = result[0]
    assert dist == pytest.approx(2.0)
    assert np.array_equal(point, key)

def test_knearest_multiple_point(tree: KDTree[float, None]) -> None:
    p1 = np.array([2.0, -2.0])
    p2 = np.array([0.0, 0.0])
    p3 = np.array([-1.5, 3.0])
    p4 = np.array([4.0, -3.0])
    p5 = np.array([1.0, 1.0])

    for pt in [p1, p2, p3, p4, p5]:
        tree.add(pt)

    query = np.array([0.5, 1.0])

    result = tree.knearest(query, k=1)

    assert len(result) == 1

    dist, key, _ = result[0]
    assert np.array_equal(key, p5)
    assert dist == pytest.approx(0.5)

def test_knearest_neighbourhood(tree: KDTree[float, None]) -> None:
    p1 = np.array([2.0, -2.0])
    p2 = np.array([0.0, 0.0])
    p3 = np.array([-1.5, 3.0])
    p4 = np.array([4.0, -3.0])
    p5 = np.array([1.0, 1.0])

    for pt in [p1, p2, p3, p4, p5]:
        tree.add(pt)

    query = np.array([-0.5, 1.0])

    result = tree.knearest(query, k=2)

    assert len(result) == 2
    
    dist, key, _ = result[0]
    assert np.array_equal(key, p2)
    assert dist == pytest.approx(1.118033)

    dist, key, _ = result[1]
    assert np.array_equal(key, p5)
    assert dist == pytest.approx(1.5)

def test_within(tree: KDTree[float, None]) -> None:
    p1 = np.array([3.0, -2.0])
    p2 = np.array([1.0, 0.2])
    p3 = np.array([-11.5, -1.0])
    p4 = np.array([10.0, 8.5])

    for pt in [p1, p2, p3, p4]:
        tree.add(pt)

    query = np.array([2.0, 0.5])

    result = tree.within(query, 5.0)
    
    # Two neighbours found, in order of closeness
    assert len(result) == 2
    assert np.array_equal(p2, result[0][1])
    assert np.array_equal(p1, result[1][1])

def test_kdtree_add_structure(tree: KDTree[float, None]) -> None:
    p1 = np.array([3,2])
    p2 = np.array([0,1])
    p3 = np.array([-1, -1])
    p4 = np.array([-2, 1.5])
    p5 = np.array([0.5, -0.5])

    for pt in [p1, p2, p3, p4, p5]:
        tree.add(pt)

    # Check correct structure based off insertion order
    root = tree._root       # p1
    n1 = root.left
    n2 = n1.left
    n3 = n1.right
    n4 = n2.right

    assert np.array_equal(root.entry.key, p1)
    assert np.array_equal(n1.entry.key, p2)
    assert np.array_equal(n2.entry.key, p3)
    assert np.array_equal(n3.entry.key, p4)
    assert np.array_equal(n4.entry.key, p5)

    assert root.right is None
    assert n2.left is None
    assert n3.left is None and n3.right is None
    assert n4.left is None and n4.right is None

def test_kdtree_rebuild(tree: KDTree[float, None]) -> None:
    p1 = np.array([3,2])
    p2 = np.array([0,1])
    p3 = np.array([-1, -1])
    p4 = np.array([-2, 1.5])
    p5 = np.array([0.5, -0.5])

    for pt in [p1, p2, p3, p4, p5]:
        tree.add(pt)

    tree.rebuild()

    root = tree._root       # p2
    n1 = root.left          # p4
    n2 = n1.left            # p3
    n3 = root.right         # p1
    n4 = n3.left            # p5

    assert np.array_equal(root.entry.key, p2)
    assert np.array_equal(n1.entry.key, p4)
    assert np.array_equal(n2.entry.key, p3)
    assert np.array_equal(n3.entry.key, p1)
    assert np.array_equal(n4.entry.key, p5)
