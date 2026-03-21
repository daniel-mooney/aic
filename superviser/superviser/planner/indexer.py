from __future__ import annotations
from abc import ABC, abstractmethod
import math
from typing import Callable, Generic, Optional, TypeVar, Protocol
from dataclasses import dataclass
import heapq

K = TypeVar('K', bound="SupportsOrder", covariant=True)
T = TypeVar('T')

class SupportsOrder(Protocol):
    """A generic variable which supports comparison."""
    def __lt__(self: K, other: K) -> bool: ...
    def __le__(self: K, other: K) -> bool: ...
    def __gt__(self: K, other: K) -> bool: ...
    def __ge__(self: K, other: K) -> bool: ...
    def __eq__(self: K, other: K, /) -> bool: ...

class Indexable(Protocol[K]):
    def __getitem__(self, index: int) -> K: ...
    def __len__(self) -> int: ...

class Indexer(ABC, Generic[K,T]):
    """An indexer into a set of items. It is used to efficiently find
    nearest-neighbours queries within the set."""
    @abstractmethod
    def add(self, key: Indexable[K], data: Optional[T] = None) -> None:
        """Add an item to the indexer

        Args:
            key:
            data: 
        """
        pass

    def clear(self) -> None:
        """Clears the index"""

    @abstractmethod
    def distance(self, a: Indexable[K], b: Indexable[K]) -> float:
        """Returns the distance between two indexable items."""
        pass

    @abstractmethod
    def knearest(
        self,
        key: Indexable[K],
        k: int = 1,
    ) -> list[tuple[float, Indexable[K], Optional[T]]]:
        """Find the k-nearest neighbour of a key

        Args:
            key: 

        Returns:
            An ordered list of the neighbourhood.
        """
        pass

    @abstractmethod
    def within(
        self,
        key: Indexable[K],
        dist: float,
    ) -> list[tuple[Indexable[K],Optional[T]]]:
        """Gets all neighbours within a specified distance to a key.

        Args:
            key: 
            dist: 
            fn: An optional distance funciton. L2-norm is used by default.

        Returns:
            
        """
        pass

@dataclass
class _KDEntry(Generic[K,T]):
    """
    Attributes:
        key: An indexable key whose elements support comparison
        data: Optional data payload which is not used in comparison.
    """
    key: Indexable[K]
    data: Optional[T]

@dataclass
class _KDNode(Generic[K,T]):
    """A node in a kd-tree"""
    entry: _KDEntry[K,T]
    axis: int
    left: Optional[_KDNode[K,T]]
    right: Optional[_KDNode[K,T]]

class KDTree(Indexer[K,T]):
    def __init__(
        self,
        dim: int,
        distance_metric: Callable[[Indexable[K], Indexable[K]], float],
        axis_dist_metric: Callable[[K, K, int], float],
    ) -> None:
        if dim <= 0:
            raise ValueError("Invalid KD-Tree Dimension")

        self._entries: list[_KDEntry[K, T]] = []
        self._metric = distance_metric
        self._axis_metric = axis_dist_metric

        self._root: Optional[_KDNode[K, T]] = None
        self._dim = dim

    def distance(self, a: Indexable[K], b: Indexable[K]) -> float:
        """Returns the distance between two indexable items."""
        return self._metric(a, b)

    def add(self, key: Indexable[K], data: Optional[T] = None) -> None:
        """Add an entry to the kd-tree.

        Args:
            key: 
            data: 

        Raises:
            ValueError: 
        """
        if len(key) != self._dim:
            raise ValueError("Key and KD-Tree dimensions do no match")

        entry = _KDEntry(key, data)
        if not self._insert(entry):
            return

        self._entries.append(entry)

    def clear(self) -> None:
        self._entries = []
        self._root = None

    def knearest(
        self,
        key: Indexable[K],
        k: int = 1,
    ) -> list[tuple[float, Indexable[K], Optional[T]]]:
        """Returns the k-nearest neighbours to the provided key.

        Args:
            key: 
            k: 

        Returns:
            The k-nearest neighbours, sorted in order of increasing distance.

        Raises:
            ValueError: 
        """
        if len(key) != self._dim:
            raise ValueError("Invalid key dimension")
        if k <= 0:
            return []

        heap: list[tuple[float, int, _KDEntry[K, T]]] = []
        self._knn_search(self._root, key, k, heap)

        # Convert max-heap (min with neg, to positive values)
        result = [(-dist, e.key, e.data) for (dist, _, e) in heap]
        result.sort(key=lambda x: x[0])

        return result

    def within(
        self,
        key: Indexable[K],
        dist_thresh: float,
    ) -> list[tuple[float, Indexable[K], Optional[T]]]:
        """Retrieves all entries that are within a given distance of a query key.

        Args:
            key: 
            dist_thresh: 

        Returns:
            
        """
        heap: list[tuple[float, int, _KDEntry]] = []
        self._within_search(self._root, key, dist_thresh, heap)

        return [(dist, entry.key, entry.data) for (dist, _, entry) in heap]

    def rebuild(self) -> None:
        """Rebuilds the kd-tree"""
        self._root = self._build_tree(self._entries, axis=0)

    def _build_tree(self, entries: list[_KDEntry[K, T]], axis: int) -> Optional[_KDNode]:
        """Constructs a kd-tree from a list of KD entries

        Args:
            entries: 
            axis: 

        Returns:
            The root of the tree given the list of entries.
        """
        if not entries:
            return None

        # Find the median of the selected axis
        entries.sort(key=lambda x: x.key[axis])
        
        mid = len(entries) // 2
        pivot = entries[mid]

        next_axis = (axis + 1) % self._dim

        return _KDNode(
            entry=pivot,
            axis=axis,
            left=self._build_tree(entries[:mid], next_axis),
            right=self._build_tree(entries[mid+1:], next_axis),
        )

    def _insert(self, entry: _KDEntry[K,T]) -> bool:
        """Inserts an entry into the kd-tree
        
        Returns:
            bool: True on successful insertion
        """
        new_node = _KDNode(entry, -1, None, None)
        
        if not self._root:
            new_node.axis = 0
            self._root = new_node

            return True

        # Find leaf node to insert at
        curr = self._root

        while curr:
            if new_node.entry.key[curr.axis] < curr.entry.key[curr.axis]:
                if not curr.left:
                    # Insert the node as left child
                    new_node.axis = (curr.axis + 1) % self._dim
                    curr.left = new_node
                    break
                else:
                    curr = curr.left
            # elif curr.entry.key == new_node.entry.key:
            #     # Do not insert duplicate keys
            #     return False
            else:
                if not curr.right:
                    # Insert the node as right child
                    new_node.axis = (curr.axis + 1) % self._dim
                    curr.right = new_node
                    break
                else:
                    curr = curr.right

        return True

    def _knn_push(
        self,
        heap: list[tuple[float, int, _KDEntry[K, T]]],
        k: int,
        entry: _KDEntry[K, T],
        dist: float,
    ) -> None:
        """Manages the knn heap"""
        if len(heap) < k:
            heapq.heappush(heap, (-dist, id(entry), entry))
        elif -dist > heap[0][0]:
            heapq.heapreplace(heap, (-dist, id(entry), entry))

    def _knn_search(
        self,
        node: Optional[_KDNode[K,T]],
        key: Indexable[K],
        k: int,
        heap: list[tuple[float, int, _KDEntry[K, T]]],
    ) -> None:
        if not node:
            return

        # Visit current node
        dist = self._metric(key, node.entry.key)
        self._knn_push(heap, k, node.entry, dist)

        # check down one branch of the tree first, then on the way up check boundaries
        axis = node.axis
        (near, far) = (node.left, node.right) if key[axis] < node.entry.key[axis] else (node.right, node.left)

        self._knn_search(near, key, k, heap)

        # See if need to check otherside of boundary (prune)
        n_axis = node.entry.key[axis]
        k_axis = key[axis]

        boundary_dist = self._axis_metric(n_axis, k_axis, axis)
        worst_dist = -heap[0][0] if heap else math.inf

        if boundary_dist < worst_dist or len(heap) < k:
            self._knn_search(far, key, k, heap)

    def _within_search(
        self,
        node: Optional[_KDNode[K, T]],
        key: Indexable[K],
        dist_thresh: float,
        heap: list[tuple[float, int, _KDEntry[K, T]]],
    ) -> None:
        if not node:
            return

        # In-order traversal
        axis = node.axis
        (near, far) = (node.left, node.right) if key[axis] < node.entry.key[axis] else (node.right, node.left)

        self._within_search(near, key, dist_thresh, heap)

        # Add current node to list?
        dist = self._metric(key, node.entry.key)
        
        if dist <= dist_thresh:
            # Min-heap
            heapq.heappush(heap, (dist, id(node.entry), node.entry))

        boundary_dist = self._axis_metric(node.entry.key[axis], key[axis], axis)

        if boundary_dist <= dist_thresh:
            self._within_search(far, key, dist_thresh, heap)
