from dataclasses import dataclass, field
from typing import Any, Dict, Type, TypeVar, Optional, Callable, Protocol

import numpy as np
from numpy.typing import NDArray

from backend.generated.thrift.config.common.ttypes import GenericVector
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType

from backend.python.pos_extrapolator.filter_strat import GenericFilterStrategy

T = TypeVar("T")
C = TypeVar("C", covariant=True)


@dataclass
class KalmanFilterInput:
    input: NDArray[np.float64] | GenericVector
    sensor_id: str
    sensor_type: KalmanFilterSensorType
    R_mult: float = 1.0
    R_add: float = 0.0
    jacobian_h: Callable[[NDArray[np.float64]], NDArray[np.float64]] | None = None
    hx: Callable[[NDArray[np.float64]], NDArray[np.float64]] | None = None

    def get_input(self) -> NDArray[np.float64]:
        if isinstance(self.input, GenericVector):
            return np.array(self.input.values)
        return self.input.copy()


@dataclass
class ExtrapolationContext:
    """
    Context object for extrapolation. This class is inputted into the prepare_data method of the DataPreparerManager.
    """

    filter: GenericFilterStrategy
    has_gotten_rotation: bool


@dataclass
class ConfigProvider(Protocol[C]):
    """
    Protocol class for providing a configuration object to DataPreparers.

    Classes inheriting from ConfigProvider are expected to hold and provide
    access to a configuration instance, which is used for controlling filter
    input preparation and behavior.
    """

    config: C

    def __init__(self, config: C):
        self.config = config

    def get_config(self) -> C:
        return self.config


class DataPreparer(Protocol[T, C]):
    def __init__(self, config: C | None = None):
        pass

    def _prepare(
        self, data: T, sensor_id: str, context: ExtrapolationContext | None = None
    ) -> list[KalmanFilterInput] | KalmanFilterInput | None:
        """
        Prepares the data for the filter. Returns a list of KalmanFilterInputs if the preparer returns a list,
        a single KalmanFilterInput if the preparer returns a single input, or None if the preparer returns None.
        """
        ...

    def prepare_input(
        self, data: T, sensor_id: str, context: ExtrapolationContext | None = None
    ) -> list[KalmanFilterInput] | None:
        """
        Prepares the data for the filter and automatically converts the result to a standardized list of KalmanFilterInputs.
        """

        result = self._prepare(data, sensor_id, context)
        if result is None:
            return None
        if isinstance(result, KalmanFilterInput):
            return [result]
        return result

    def _used_indices(self, *args, **kwargs) -> list[bool]:
        """
        Returns the used indices for the preparer.
        Example: [True, True, True, True, True, True] for a preparer that uses all states.
        """
        ...

    def get_used_indices(self, *args, **kwargs) -> list[bool]:
        """
        Returns the used indices for the preparer, checking that the number of indices is
        correct and matches the number of states in the filter (GenericFilterStrategy.kNumStates).
        """

        indices = self._used_indices(*args, **kwargs)

        if len(indices) != GenericFilterStrategy.kNumStates:
            raise ValueError(
                f"Expected {GenericFilterStrategy.kNumStates} indices, got {len(indices)}"
            )

        return indices

    def get_data_type(self) -> type[T]: ...


class DataPreparerManager:
    _registry: dict[str, type[DataPreparer[Any, Any]]] = {}
    _config_instances: dict[str, Any] = {}

    @classmethod
    def register(cls, proto_type: Type[T], config_instance: Any = None):
        """
        Register a DataPreparer class for a given proto_type.

        This allows the DataPreparerManager to know which DataPreparer should be used
        for a specific protobuf data type.

        Optionally, an initial config_instance can be set for the corresponding proto_type.
        """

        def decorator(preparer_class: Type[DataPreparer[T, Any]]):
            cls._registry[proto_type.__name__] = preparer_class
            if config_instance is not None:
                cls._config_instances[proto_type.__name__] = config_instance
            return preparer_class

        return decorator

    @classmethod
    def set_config(cls, proto_type: type[T], config_instance: Any):
        """
        Sets the configuration instance associated with the given proto_type in the DataPreparerManager.

        This method is used to specify which configuration object should be used by preparers registered for a particular data type.
        The configuration will be passed to preparer instances when preparing input data.

        Args:
            proto_type: The protobuf type for which the configuration is being set.
            config_instance: The configuration object to associate with the proto_type.
        """
        cls._config_instances[proto_type.__name__] = config_instance

    def prepare_data(
        self, data: object, sensor_id: str, context: ExtrapolationContext | None = None
    ) -> list[KalmanFilterInput] | None:
        """
        Prepares the data for the filter.

        Parameters:
            data: The data to prepare. This is a protobuf object (NOT bytes).
            sensor_id: The id of the sensor that the data is from.
            context: The context for the extrapolation.

        Returns:
            A list of KalmanFilterInputs if the preparer returns a list, a single KalmanFilterInput if the preparer returns a single input, or None if the preparer returns None.
        """

        data_type_name = type(data).__name__

        if data_type_name not in self._registry:
            return None

        preparer_class = self._registry[data_type_name]
        config_instance = self._config_instances.get(data_type_name)
        preparer_instance = preparer_class(config_instance)

        return preparer_instance.prepare_input(data, sensor_id, context)
