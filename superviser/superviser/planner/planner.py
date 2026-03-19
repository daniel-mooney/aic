from abc import ABC, abstractmethod
from typing import List
from planner.types import State

from planner.collision_checker import CollisionChecker
from planner.sampler import StateSampler
from planner.indexer import Indexer

class Planner(ABC):
    @abstractmethod
    def plan(self, start: State, goal: State) -> List[State]:
        pass

class RRT(Planner):
    def __init__(
        self,
        collision_checker: CollisionChecker,
        sampler: StateSampler,
        indexer: Indexer,
    ) -> None:
        self._collision_checker = collision_checker
        self._sampler = sampler
        self._indexer = indexer

    def plan(self, start: State, goal: State) -> List[State]:
        return super().plan(start, goal)

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

    def plan(self, start: State, goal: State) -> List[State]:
        return super().plan(start, goal)
