from abc import ABC, abstractmethod
from superviser.planner.types import State
import numpy as np

class Obstacle(ABC):
    @abstractmethod
    def contains(self, state: State) -> bool:
        """Check if a configuration is contained within an Obstacle.

        Args:
            cfg: The configuration to check

        Returns: True if the configuration is within the obstacle.
            
        """
        pass

    @abstractmethod
    def contains_line(self, a: State, b: State) -> bool:
        """Check if the line segment between two states intersects with
        the obstacle

        Args:
            a: One end of the line segment.
            b: The other end of the line segment.

        Returns:
            bool: True if the line segment intersects with the obstacle. 
        """
        pass

    @property
    @abstractmethod
    def dim(self) -> int:
        """The dimension of the obstacle"""
        pass

    @abstractmethod
    def collision_interval(self, a: State, b: State) -> float:
        """Returns the percentage of an interval until a collision
        occurs with the given obstacle. The output will always fall in [0,1],
        where 0 indicates `a` being in a collision and 1 for no collision along
        the entire interval."""
        pass

class RectangularObstacle3d(Obstacle):
    def __init__(self, x_span: tuple, y_span: tuple, z_span: tuple) -> None:
        """Creates a Rectangular Obstacle.

        Spans are specified in absolute coordinates as [min, max] and are inclusive.
        Args:
            x_span:
            y_span:
            z_span: 
        """
        self._x_span = x_span
        self._y_span = y_span
        self._z_span = z_span

    def contains(self, cfg: State) -> bool:
        if len(cfg) != self.dim:
            raise ValueError("Invalid state dimension")

        return (
            self._x_span[0] <= cfg[0] <= self._x_span[1]
            and self._y_span[0] <= cfg[1] <= self._y_span[1]
            and self._z_span[0] <= cfg[2] <= self._z_span[1]
        )

    def contains_line(self, a: State, b: State) -> bool:
        return self.collision_interval(a, b) != 1

    
    def collision_interval(self, a: State, b: State) -> float:
        if len(a) != self.dim or len(b) != self.dim:
            raise ValueError("Invalid state dimension")

        # Slab method
        with np.errstate(divide='ignore', invalid='ignore'):
            diff = b - a
            
            (x_low, x_high) = self._x_span
            (y_low, y_high) = self._y_span
            (z_low, z_high) = self._z_span
            
            # Use np.divide for IEEE754 i.e. extended real numbers
            tx_low = np.divide(x_low - a[0], diff[0])
            tx_high = np.divide(x_high - a[0], diff[0])
            tx_close = np.fmin(tx_low, tx_high)
            tx_far = np.fmax(tx_low, tx_high)

            ty_low = np.divide(y_low - a[1], diff[1])
            ty_high = np.divide(y_high - a[1], diff[1])
            ty_close = np.fmin(ty_low, ty_high)
            ty_far = np.fmax(ty_low, ty_high)

            tz_low = np.divide(z_low - a[2], diff[2])
            tz_high = np.divide(z_high - a[2], diff[2])
            tz_close = np.fmin(tz_low, tz_high)
            tz_far = np.fmax(tz_low, tz_high)

            t_close = max(tx_close, ty_close, tz_close)
            t_far = min(tx_far, ty_far, tz_far)

        if t_close <= t_far and 0 <= t_close <= 1 and t_far >= 0:
            return t_close
        else:
            return 1.0

    @property
    def dim(self) -> int:
        return 3

class RectangularObstacle2d(Obstacle):
    def __init__(self, x_span: tuple, y_span: tuple) -> None:
        self._x_span = x_span
        self._y_span = y_span

    @property
    def dim(self) -> int:
        return 2

    def contains(self, state: State) -> bool:
        (x_min, x_max) = self._x_span
        (y_min, y_max) = self._y_span

        return (
            x_min <= state[0] <= x_max
            and y_min <= state[1] <= y_max
        )

    def contains_line(self, a: State, b: State) -> bool:
        return self.collision_interval(a, b) != 1

    def collision_interval(self, a: State, b: State) -> float:
        if len(a) != self.dim or len(b) != self.dim:
            raise ValueError("Invalid state dimension")

        # Slab method
        with np.errstate(divide='ignore', invalid='ignore'):
            diff = b - a
            
            (x_low, x_high) = self._x_span
            (y_low, y_high) = self._y_span
            
            # Use np.divide for IEEE754 i.e. extended real numbers
            tx_low = np.divide(x_low - a[0], diff[0])
            tx_high = np.divide(x_high - a[0], diff[0])
            tx_close = np.fmin(tx_low, tx_high)
            tx_far = np.fmax(tx_low, tx_high)

            ty_low = np.divide(y_low - a[1], diff[1])
            ty_high = np.divide(y_high - a[1], diff[1])
            ty_close = np.fmin(ty_low, ty_high)
            ty_far = np.fmax(ty_low, ty_high)

            t_close = max(tx_close, ty_close)
            t_far = min(tx_far, ty_far)

        if t_close <= t_far and 0 <= t_close <= 1 and t_far >= 0:
            return t_close
        else:
            return 1.0

class CollisionChecker(ABC):
    def __init__(self, inter_method: str = "straight") -> None:
        self._inter_method = inter_method


    @abstractmethod
    def contains_state(self, cfg: State) -> bool:
        pass

    @abstractmethod
    def contains_edge(self, a: State, b: State) -> bool:
        pass

    @abstractmethod
    def collision_interval(self, a: State, b: State) -> float:
        pass

class StraightCollisionChecker(CollisionChecker):
    def __init__(self, inter_method: str = "straight") -> None:
        """Creates a new Collision checker object
        """
        super().__init__(inter_method)
        self._obstacles: list[Obstacle] = []

    def add_obstacle(self, obs: Obstacle) -> None:
        # if obs.dim != len(self._boundary):
        #     raise RuntimeError(f"Invalid obstacle dimension for cfg space with dim: {len(self._boundary)}")

        self._obstacles.append(obs)

    def contains_state(self, cfg: State) -> bool:
        """Checks if a configuration is within an obstacle."""
        for obs in self._obstacles:
            if obs.contains(cfg):
                return True
        return False

    def contains_edge(self, a: State, b: State) -> bool:
        """Checks if a straight edge passes through an obstacle

        Args:
            a: The starting configuration. 
            b: The ending configuration.

        Returns: True if a collision occurs.
            
        """
        for obs in self._obstacles:
            if obs.contains_line(a, b):
                return True
        return False

    def collision_interval(self, a: State, b: State) -> float:
        """Finds the interval proportion until an obstacle is hit.

        Args:
            a: 
            b: 

        Returns:
            
        """
        interval = 1.0

        for obs in self._obstacles:
            interval = min(interval, obs.collision_interval(a, b))

        return interval
