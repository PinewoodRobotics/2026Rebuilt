import json
import sys
import os
from pathlib import Path

# Add src/ directory to Python path so backend imports work
script_dir = Path(__file__).parent
src_dir = script_dir.parent.parent.parent  # src/backend/python -> src/
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import requests
from backend.python.common.config import get_config_raw
from backend.python.common.util.system import load_basic_system_config

config_base64 = get_config_raw()
watchdog_port = 5000
host = "agathaking.local"
stopping_processes = ["april-server", "position-extrapolator"]
starting_processes = ["april-server"]

response = requests.post(
    f"http://{host}:{watchdog_port}/set/config",
    json={"config": config_base64},
)

print(f"{host} Lidar 3D Setting Config Output: {response.json()}")

stop_response = requests.post(
    f"http://{host}:{watchdog_port}/stop/process",
    json={"process_types": stopping_processes},
)

print(f"{host} Lidar 3D Stopping Process Output: {stop_response.json()}")

response = requests.post(
    f"http://{host}:{watchdog_port}/start/process",
    json={"process_types": starting_processes},
)

print(f"{host} Pos Extrapolator Starting Process Output: {response.json()}")
