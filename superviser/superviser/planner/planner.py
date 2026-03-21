from abc import ABC, abstractmethod
from collections import deque
import networkx as nx
import numpy as np

from superviser.planner.types import State
from superviser.planner.collision_checker import CollisionChecker
from superviser.planner.sampler import StateSampler
from superviser.planner.indexer import Indexer

class Planner(ABC):
    @abstractmethod
    def plan(self, start: State, goal: State) -> list[State]:
        """Compute a plan from a starting state to a goal state

        Args:
            start: 
            goal: 

        Returns:
            An ordered list of states, beginning with `start` and ending
            with `goal`.
        """
        pass

class RRT(Planner):
    def __init__(
        self,
        collision_checker: CollisionChecker,
        sampler: StateSampler,
        indexer: Indexer,
        max_radius: float,
    ) -> None:
        self._collision_checker = collision_checker
        self._sampler = sampler
        self._indexer = indexer
        self._max_radius = max_radius

        self._graph = nx.DiGraph()

    def plan(self, start: State, goal: State) -> list[State]:
        if self._collision_checker.contains_state(start) or self._collision_checker.contains_state(goal):
            raise ValueError("Path end points cannot be in a collision state")
        
        self._indexer.clear()
        self._graph.clear()

        self._indexer.add(start)

        # Must use tuple in graph. NDArray is not hashable
        self._graph.add_node(tuple(start))

        # Perform the RRT algorithm
        last_state = start
        dist = self._indexer.distance(last_state, goal)

        print(f"Init dist: {dist}")
        # TODO: possible max iterations
        while self._indexer.distance(last_state, goal) >= self._max_radius or self._collision_checker.contains_edge(last_state, goal):
            s = self._sampler.sample()
            dist, q, _ = self._indexer.knearest(s, k=1)[0]      # closest neighbour

            delta = s - q
            direction = delta / np.linalg.norm(delta)

            # Get direction if distance exceeds max
            if dist > self._max_radius:
                # update sampled point
                s = q + direction * self._max_radius
                delta = s - q

            # Check if hit obstacle, update sampled point if so
            max_inter = self._collision_checker.collision_interval(q, s)

            if max_inter != 1.0:
                s = q + delta * max_inter

            # Add to graph
            self._indexer.add(s)
            self._graph.add_node(tuple(s))
            self._graph.add_edge(tuple(q), tuple(s))

            last_state = s

            dist = self._indexer.distance(last_state, goal)
            collide = self._collision_checker.contains_edge(last_state, goal)
            # print(f"distance: {dist:.2f}, {collide=}", flush=True)

        self._graph.add_node(tuple(goal))
        self._graph.add_edge(tuple(last_state), tuple(goal))

        # Generate ordered list of states
        curr = tuple(goal)
        predecessors = list(self._graph.predecessors(curr))

        trajectory = deque()
        while predecessors:
            state = np.array(curr)
            trajectory.appendleft(state)

            # Only ever one predecessor i.e. parent
            curr = predecessors[0]
            predecessors = list(self._graph.predecessors(curr))

        trajectory.appendleft(start)
        return list(trajectory)

class RRTStar(Planner):
    def __init__(
        self,
        collision_checker: CollisionChecker,
        sampler: StateSampler,
        indexer: Indexer,
    ) -> None:
        self._collision_checker = collision_checker
        self._sampler = sampler
        self._indexer = indexer

    def plan(self, start: State, goal: State) -> list[State]:
        return super().plan(start, goal)
