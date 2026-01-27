from enum import Enum
from backend.deployment.system_types import SystemType
from backend.deployment.util import (
    _WeightedProcess,
    ModuleTypes,
    _Module,
    DeploymentOptions,
)
from backend.deployment.process_type_util import (
    ProcessPlan,
)


class ProcessType(_WeightedProcess):
    POS_EXTRAPOLATOR = "position-extrapolator", 0.5
    APRIL_SERVER = "april-server", 1.0
    FAN_COLOR = "fan-color", 0.1


def pi_name_to_process_types(pi_names: list[str]) -> dict[str, list[ProcessType]]:
    """Assigns processes to discovered Pis (pi_names provided at runtime)."""

    return (
        ProcessPlan[ProcessType]()
        .add(ProcessType.POS_EXTRAPOLATOR)
        .pin(ProcessType.APRIL_SERVER, "nathan-hale")
        .pin(ProcessType.APRIL_SERVER, "tynan")
        .pin(ProcessType.APRIL_SERVER, "agatha-king")
        .assign(pi_names)
    )


def get_modules() -> list[_Module]:
    return [
        ModuleTypes.ProtobufModule(
            project_root_folder_path="src/proto",
            build_for_platforms=[],
        ),
        ModuleTypes.ThriftModule(
            project_root_folder_path="ThriftTsConfig/schema",
            build_for_platforms=[],
        ),
        ModuleTypes.PythonModule(
            local_root_folder_path="python/pos_extrapolator",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition=ProcessType.POS_EXTRAPOLATOR.get_name(),
        ),
        ModuleTypes.PythonModule(
            local_root_folder_path="python/april",
            local_main_file_path="src/main.py",
            extra_run_args=[],
            equivalent_run_definition=ProcessType.APRIL_SERVER.get_name(),
        ),
        ModuleTypes.PythonModule(
            local_root_folder_path="python/fan_color",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition=ProcessType.FAN_COLOR.get_name(),
        ),
    ]


if __name__ == "__main__":
    DeploymentOptions.with_automatic_discovery(get_modules(), pi_name_to_process_types)
