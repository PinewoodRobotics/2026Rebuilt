from backend.deployment.compilation_util import CPPBuildConfig, CPPLibrary
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
    ]


if __name__ == "__main__":
    DeploymentOptions.with_automatic_discovery(get_modules(), pi_name_to_process_types)


"""
        ModuleTypes.CPPLibraryModule(
            name="cuda-tags-lib",
            project_root_folder_path="cpp/CudaTags",
            build_for_platforms=[],  # SystemType.JETPACK_L4T_R36_2
            compilation_config=CPPBuildConfig.with_cmake(
                cmake_args=[
                    "-DCUDATAGS_BUILD_PYTHON=ON",
                    "-DCMAKE_BUILD_TYPE=Release",
                    "-DPYBIND11_FINDPYTHON=ON",
                    "-DPYBIND11_PYTHON_VERSION=3.12",
                    "-DPython_EXECUTABLE=/usr/bin/python3.12",
                    "-DPython3_EXECUTABLE=/usr/bin/python3.12",
                    "-DPYTHON_EXECUTABLE=/usr/bin/python3.12",
                    "-DPython3_FIND_STRATEGY=LOCATION",
                ],
                compiler_args=[],
                libs=[
                    CPPLibrary(name="python3"),
                    CPPLibrary(name="python-is-python3"),
                    CPPLibrary(name="python3-numpy"),
                    CPPLibrary(name="python3-pip"),
                    CPPLibrary(name="libopencv-dev"),
                    CPPLibrary(name="openjdk-11-jdk"),
                    CPPLibrary(name="default-jdk"),
                    CPPLibrary(name="cmake"),
                    CPPLibrary(name="ninja-build"),
                    CPPLibrary(name="pkg-config"),
                    CPPLibrary(name="git"),
                    CPPLibrary(name="build-essential"),
                    CPPLibrary(name="libssl-dev"),
                ],
                extra_docker_commands=[
                    "apt-get update",
                    "apt-get install -y software-properties-common",
                    "add-apt-repository -y ppa:deadsnakes/ppa",
                    "apt-get update",
                    "apt-get install -y python3.12 python3.12-dev python3.12-venv",
                    "curl -sS https://bootstrap.pypa.io/get-pip.py | python3.12",
                    "python3.12 -m pip install --no-cache-dir numpy",
                    "git clone --depth 1 --branch v2.12.0 https://github.com/pybind/pybind11.git /tmp/pybind11",
                    "cmake -S /tmp/pybind11 -B /tmp/pybind11/build -DPYBIND11_TEST=OFF -DCMAKE_BUILD_TYPE=Release",
                    "cmake --build /tmp/pybind11/build -j$(nproc)",
                    "cmake --install /tmp/pybind11/build",
                ],
            ),
        ),

        ModuleTypes.PythonModule(
            local_root_folder_path="python/fan_color",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition=ProcessType.FAN_COLOR.get_name(),
        ),
        """
