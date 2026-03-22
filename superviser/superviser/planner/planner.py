from abc import ABC, abstractmethod
from collections import deque
import networkx as nx
import numpy as np
import random

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
        check_goal_rate: float = 0.05
    ) -> None:
        self._collision_checker = collision_checker
        self._sampler = sampler
        self._indexer = indexer
        self._max_radius = max_radius
        self._check_goal_rate = check_goal_rate

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

        # TODO: possible max iterations
        while self._indexer.distance(last_state, goal) >= self._max_radius or self._collision_checker.contains_edge(last_state, goal):
            # Randomly see if you can connect the goal to some node
            check_goal = random.random() < self._check_goal_rate
            if check_goal:
                dist, nearest, _ = self._indexer.knearest(goal)[0]

                if not self._collision_checker.contains_edge(nearest, goal):
                    last_state = nearest
                    break

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
        join_radius: float,
        update_radius: float,
        check_goal_rate: float = 0.05,
    ) -> None:
        self._collision_checker = collision_checker
        self._sampler = sampler
        self._indexer = indexer
        self._join_radius = join_radius
        self._update_radius = update_radius
        self._check_goal_rate = check_goal_rate
        
        self._graph = nx.DiGraph()

    def plan(self, start: State, goal: State) -> list[State]:
        if self._collision_checker.contains_state(start) or self._collision_checker.contains_state(goal):
            raise ValueError("Path end points cannot be in a collision state")
        
        self._indexer.clear()
        self._graph.clear()

        self._indexer.add(start)
        self._graph.add_node(tuple(start), cost=0)

        last_state = start
        last_dist = 0
        while True:
            # Randomly check if can connect to goal
            check_goal = random.random() < self._check_goal_rate
            # check_goal = False
            if check_goal:
                # Can assume indexer has at least one item
                n_dist, nearest, _ = self._indexer.knearest(goal)[0]

                if not self._collision_checker.contains_edge(nearest, goal):
                    last_state = nearest
                    last_dist = n_dist
                    break
            
            s = self._sampler.sample()

            # Use knearest to generate a sample within join radius.
            # Join to the neighbour resulting in the lowest cost to go.
            dist, q, _ = self._indexer.knearest(s, k=1)[0]
            delta = s - q
            direction = delta / np.linalg.norm(delta)

            # Cap join distance
            if dist > self._join_radius:
                s = q + direction * self._join_radius
                delta = s - q

            # Check if edge goes through obstacle, update edge if so
            max_iter = self._collision_checker.collision_interval(q, s)

            if max_iter != 1.0:
                s = q + delta * max_iter

            if tuple(s) in self._graph:
                continue

            s_cost = self._add_node(s)
            
            last_state = s
            last_cost = s_cost
            # See if can terminate
            if self._indexer.distance(s, goal) < self._join_radius and not self._collision_checker.contains_edge(s, goal):
                last_dist = self._indexer.distance(s, goal)
                break

        # Connect last state to goal
        last_cost = self._graph.nodes[tuple(last_state)]['cost']
        self._graph.add_node(tuple(goal), cost=(last_cost + last_dist))
        self._graph.add_edge(tuple(last_state), tuple(goal))
        # self._add_node(goal)

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

    def _add_node(self, s: State) -> float:
        """Adds the node to the graph. Returns the cost to reach the node"""
        # Join to best neighbour
        neighbours = [p for p, _ in self._indexer.within(s, self._update_radius)]

        # Filter out neighbours that result in collision
        neighbours = [x for x in neighbours if not self._collision_checker.contains_edge(s, x)]

        cost_to_reach = lambda p: (
                self._graph.nodes[tuple(p)]['cost'] 
                + self._indexer.distance(p, s)
                )
        best_neighbour = min(neighbours, key=cost_to_reach)

        # Add to graph
        s_cost = self._indexer.distance(best_neighbour, s) + self._graph.nodes[tuple(best_neighbour)]['cost'] 
        self._graph.add_node(tuple(s), cost=s_cost)
        self._graph.add_edge(tuple(best_neighbour), tuple(s))
        self._indexer.add(s)

        # Improve neighbour edges
        for n in neighbours:
            n_node = self._graph.nodes[tuple(n)]
            n_cost = n_node['cost']
            edge_cost = self._indexer.distance(s, n)

            # Replace edge if better path through s
            if n_cost > s_cost + edge_cost:
                # Only one parent per node
                p_key, n_key = next(iter(self._graph.in_edges(tuple(n))))
                self._graph.remove_edge(p_key, n_key)

                self._graph.add_edge(tuple(s), n_key)
                delta_cost = (s_cost + edge_cost) - n_cost
                # n_node['cost'] = edge_cost + s_cost
                self._update_costs(n_key, delta_cost)

    def _update_costs(self, root: tuple, delta: float) -> None:
        stack = [root]

        while stack:
            node = stack.pop()
            self._graph.nodes[node]['cost'] += delta
            stack.extend(self._graph.successors(node))
