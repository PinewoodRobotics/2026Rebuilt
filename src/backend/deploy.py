from backend.deployment.util import (
    ModuleTypes,
    _Module,
    DeploymentOptions,
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
            equivalent_run_definition="position-extrapolator",
        ),
        ModuleTypes.PythonModule(
            local_root_folder_path="python/april",
            local_main_file_path="src/main.py",
            extra_run_args=[],
            equivalent_run_definition="april-server",
        ),
        ModuleTypes.PythonModule(
            local_root_folder_path="python/fan_color",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="fan-color",
        ),
    ]

    """
    This module requires a build (docker). All three require some specific configuration related to naming.

    ModuleTypes.CPPLibraryModule()
    ModuleTypes.CPPRunnableModule()
    ModuleTypes.RustModule()
    """

    """
    This module does not require build or docker.
    
    Example python process. This would go into python/pos_extrapolator/ folder
    and would have a main.py file that would be runnable and would start an inf loop 
    that would publish the position to the network.

    ModuleTypes.PythonModule(
        local_root_folder_path="python/pos_extrapolator",
        local_main_file_path="main.py",
        extra_run_args=[],
        equivalent_run_definition="position-extrapolator",
    ),
    """


if __name__ == "__main__":
    # Uncomment the options you want to use. These are all optional and need to be placed before the automatic discovery.
    # DeploymentOptions.without_rebuilding_binaries()
    # DeploymentOptions.with_exclude_cpp_dir(True)
    # DeploymentOptions.with_custom_backend_dir("~/Documents/B.L.I.T.Z/backend")
    # DeploymentOptions.with_preset_pi_addresses(
    #     [RaspberryPi(address="localhost", port=2222)], get_modules()
    # )
    # DeploymentOptions.with_exclusions_from_gitignore("~/Documents/B.L.I.T.Z/backend/.gitignore")
    # DeploymentOptions.with_discovery_timeout(10.0)

    DeploymentOptions.with_automatic_discovery(get_modules())
