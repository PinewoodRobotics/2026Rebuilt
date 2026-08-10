import argparse
from enum import Enum
import os
import subprocess
import sys
from types import ModuleType
import psutil
import json
import re
from pydantic import BaseModel
import netifaces
import socket
import os
import platform

from backend.python.common.config import from_uncertainty_config
from backend.generated.thrift.config.ttypes import Config
import importlib
import importlib.util
import importlib.machinery

self_name: None | str = None


class ProcessType(Enum):
    POS_EXTRAPOLATOR = "position-extrapolator"
    LIDAR_READER_2D = "lidar-reader-2d"
    LIDAR_POINT_PROCESSOR = "lidar-point-processor"
    LIDAR_PROCESSING = "lidar-processing"
    CAMERA_PROCESSING = "april-server"
    LIDAR_3D = "lidar-3d"


class AutobahnBaseConfig(BaseModel):
    host: str
    port: int


class GlobalLoggingBaseConfig(BaseModel):
    global_log_pub_topic: str
    global_logging_publishing_enabled: bool
    global_logging_level: str


class WatchdogBaseConfig(BaseModel):
    host: str
    port: int
    stats_pub_period_s: float
    send_stats: bool
    process_memory_file: str


class BasicSystemConfig(BaseModel):
    autobahn: AutobahnBaseConfig
    logging: GlobalLoggingBaseConfig
    watchdog: WatchdogBaseConfig
    config_path: str


class SystemStatus(Enum):
    PRODUCTION = "production"
    SIMULATION = "simulation"


def get_system_status() -> SystemStatus:
    return SystemStatus.PRODUCTION


def get_top_10_processes() -> list[psutil.Process]:
    processes = sorted(
        [
            p
            for p in psutil.process_iter(attrs=["pid", "name", "cpu_percent"])
            if p.info["cpu_percent"] is not None
        ],
        key=lambda p: p.info["cpu_percent"],
        reverse=True,
    )

    return processes[:10]


def get_local_ip(iface: str = "eth0") -> str | None:
    """
    Returns the IPv4 address for the given interface (e.g. "eth0" or "en0"),
    or None if the interface isn't found or has no IPv4 address.
    """
    try:
        addrs = netifaces.ifaddresses(iface)
        ipv4 = addrs.get(netifaces.AF_INET, [])
        if ipv4 and "addr" in ipv4[0]:
            return ipv4[0]["addr"]
    except ValueError:
        pass
    return None


def get_local_hostname(include_local_suffix: bool = True) -> str:
    hostname = socket.gethostname()
    if include_local_suffix and not hostname.endswith(".local"):
        return f"{hostname}.local"
    return hostname


def get_config_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-file-path", type=str, default=None)
    parser.add_argument("--name-file-path", type=str, default=None)
    parser.add_argument("--basic-system-config-file-path", type=str, default=None)
    return parser


def get_system_name(args: argparse.Namespace | None = None) -> str:
    global self_name
    if args is None:
        args, _ = get_config_parser().parse_known_args()
    if self_name is None:
        with open(args.name_file_path, "r") as f:
            self_name = f.read().strip()

    return self_name


def load_basic_system_config(
    args: argparse.Namespace | None = None,
) -> BasicSystemConfig:
    if args is None:
        args, _ = get_config_parser().parse_known_args()
    system_name = get_system_name(args)

    with open(args.basic_system_config_file_path, "r") as f:
        config_content = f.read()

    config_content = re.sub(r"<system_name>", system_name, config_content)

    config_dict = json.loads(config_content)
    return BasicSystemConfig(**config_dict)


def load_configs() -> tuple[BasicSystemConfig, Config]:
    args, _ = get_config_parser().parse_known_args()
    basic_system_config = load_basic_system_config(args)
    config = from_uncertainty_config(args.config_file_path)
    if config is None or basic_system_config is None:
        raise ValueError("Failed to load configs")

    return basic_system_config, config


def get_glibc_version() -> str:
    """
    Returns the system's glibc version string, e.g., "2.35-0ubuntu3.8"
    Strips any extraneous parentheses or trailing characters such as ')'.
    In the special case of ldd (Ubuntu GLIBC 2.35-0ubuntu3.8) 2.35,
    will return just "2.35".
    """
    import re

    try:
        output = subprocess.check_output(
            ["ldd", "--version"], encoding="utf-8", errors="ignore"
        )
        lines = output.splitlines()

        # Preferred pattern: match ldd (...) <version>
        for line in lines:
            # Pattern 1: ldd (Ubuntu GLIBC 2.35-0ubuntu3.8) 2.35
            #             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  ^^^^
            m = re.match(r"^ldd\s+\((.*?)\)\s+([0-9\.]+)", line)
            if m:
                # group(2) is the version after the paren
                return m.group(2)

            # Pattern 2: 'GLIBC 2.35-0ubuntu3.8'
            m2 = re.search(
                r"(?:GLIBC|GNU libc)[^\d]*([0-9]+(?:\.[0-9]+)*(?:-[\w\.]+)?)", line
            )
            if m2:
                # Only keep the pure version number:
                # If m2.group(1) looks like '2.35-0ubuntu3.8', try to extract the major.minor part
                version = m2.group(1)
                # Extract first digit dot digit pattern
                core = re.match(r"^([0-9]+\.[0-9]+)", version)
                if core:
                    return core.group(1)
                return version

            # Pattern 3: fallback paren group with version inside
            m3 = re.search(r"\(([^)]*\d[^)]*)\)", line)
            if m3:
                inner = m3.group(1)
                for piece in inner.split():
                    # Find the first piece that looks like a version
                    core = re.match(r"^([0-9]+\.[0-9]+)", piece)
                    if core:
                        return core.group(1)
                    if any(ch.isdigit() for ch in piece):
                        return piece.rstrip(")")
                # If nothing else, just return the whole group
                return inner.rstrip(")")

        # Final fallback: scan all words in first line for digit-dot-digit pattern
        if lines:
            for word in lines[0].split():
                core = re.match(r"^([0-9]+\.[0-9]+)", word)
                if core:
                    return core.group(1)
                if any(ch.isdigit() for ch in word):
                    return word.rstrip(")")

    except Exception:
        pass
    # Try libc.so.6 version as last resort
    try:
        import ctypes

        libc = ctypes.CDLL("libc.so.6")
        get_ver = libc.gnu_get_libc_version
        get_ver.restype = ctypes.c_char_p
        return get_ver().decode("utf-8")
    except Exception:
        pass
    raise RuntimeError("Could not determine glibc version")


def get_local_binary_path() -> str:
    """
    Returns the path to the local binary directory based on the detected C library (glibc) version and system architecture.
    Example: /opt/blitz/B.L.I.T.Z/build/release/2.35/aarch64/
    """
    clib_version = get_glibc_version()
    arch = platform.machine()  # e.g., 'x86_64', 'aarch64'
    path = f"/opt/blitz/B.L.I.T.Z/build/release/{clib_version}/{arch}/"
    return path


def setup_shared_library_python_extension(
    *,
    module_name: str,
    py_lib_searchpath: str,
    module_basename: str,
) -> ModuleType:
    binary_path = get_local_binary_path()
    print(f"[Loader] binary_path: {binary_path}")

    search_path = os.path.join(binary_path, str(py_lib_searchpath))
    dir_path = search_path
    explicit_extension_file: str | None = None

    # `py_lib_searchpath` is typically a directory like "cpp/cuda-tags-lib/".
    # Keep that directory, do not strip to parent.
    if os.path.isfile(search_path):
        explicit_extension_file = search_path
        dir_path = os.path.dirname(search_path)

    print(f"[Loader] module_parent: {dir_path}")

    if dir_path not in sys.path:
        sys.path.insert(0, dir_path)
        print(f"[Loader] Added '{dir_path}' to sys.path")

    # Ensure native dependency lookup includes the module folder.
    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_entries = [p for p in current_ld_library_path.split(":") if p]
    if dir_path not in ld_entries:
        os.environ["LD_LIBRARY_PATH"] = ":".join([dir_path, *ld_entries])
        print(f"[Loader] Prepended '{dir_path}' to LD_LIBRARY_PATH")

    print(f"[Loader] module_search_path: {search_path}")

    extension_file: str | None = None

    print(f"[Loader] dir_path: {dir_path}, base_stem: {module_basename}")

    if explicit_extension_file is not None:
        extension_file = explicit_extension_file
        print(f"[Loader] Using explicit extension file: {extension_file}")
    elif os.path.isdir(dir_path):
        candidates: list[str] = []
        valid_suffixes = tuple(importlib.machinery.EXTENSION_SUFFIXES)
        for fname in os.listdir(dir_path):
            print(f"[Loader] Candidate extension file: {fname}")
            if (
                fname.startswith(module_basename)
                and (fname.endswith(".so") or fname.endswith(".pyd"))
                and os.path.isfile(os.path.join(dir_path, fname))
            ):
                # Only accept extension suffixes compatible with the current interpreter ABI.
                if fname.endswith(valid_suffixes):
                    candidates.append(os.path.join(dir_path, fname))
                else:
                    print(
                        f"[Loader] Skipping incompatible extension suffix for current Python: {fname}"
                    )

        def suffix_rank(path: str) -> int:
            # Prefer the most specific suffix for the running Python (e.g. cpython-312...).
            name = os.path.basename(path)
            for idx, suffix in enumerate(importlib.machinery.EXTENSION_SUFFIXES):
                if name.endswith(suffix):
                    return idx
            return len(importlib.machinery.EXTENSION_SUFFIXES)

        if candidates:
            candidates.sort(key=suffix_rank)
            extension_file = candidates[0]
            print(f"[Loader] Selected extension_file: {extension_file}")
    else:
        print(f"[Loader] WARNING: Directory '{dir_path}' does not exist")

    print(f"[Loader] extension_file to import: {extension_file}")
    if extension_file is None:
        py_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
        raise ImportError(
            f"Could not find compatible extension module '{module_basename}' in '{dir_path}' "
            f"for Python {py_ver}. Compatible suffixes: {importlib.machinery.EXTENSION_SUFFIXES}"
        )
    spec = importlib.util.spec_from_file_location(module_name, extension_file)
    if spec is None or spec.loader is None:
        raise ImportError(
            f"Failed to create spec for {module_name} from {extension_file}"
        )
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as e:
        ldd_missing_lines: list[str] = []
        try:
            ldd_output = subprocess.check_output(
                ["ldd", extension_file], encoding="utf-8", errors="ignore"
            )
            for line in ldd_output.splitlines():
                if "not found" in line:
                    ldd_missing_lines.append(line.strip())
        except Exception:
            pass

        if ldd_missing_lines:
            raise ImportError(
                f"{e}. Missing shared library deps for {extension_file}: "
                + "; ".join(ldd_missing_lines)
            ) from e
        raise
    return module
