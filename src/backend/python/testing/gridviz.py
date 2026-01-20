# Add src/ directory to Python path so backend imports work
import sys
import os
from pathlib import Path

# --- FIX block (1-5): ensure src/ is in sys.path for all call locations
here = Path(__file__).resolve()
src_dir = here.parents[3]  # src/backend/python/testing/gridviz.py -> src/
src_dir_str = str(src_dir)
if src_dir_str not in sys.path:
    sys.path.insert(0, src_dir_str)
os.environ["PYTHONPATH"] = src_dir_str + (
    ":" + os.environ["PYTHONPATH"] if "PYTHONPATH" in os.environ else ""
)

import asyncio
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from autobahn_client.util import Address
from backend.generated.proto.python.pathing.pathfind_pb2 import (
    Grid,
    Cell,
)
from autobahn_client.client import Autobahn

# Global state for the plot
fig = None
ax = None
im = None
current_grid_data = None
lock = asyncio.Lock()

# Color mapping: PASSABLE=0 (white), OBSTACLE=1 (black), TEMPORARY=2 (yellow)
colors = ["white", "black", "yellow"]
cmap = ListedColormap(colors)


def update_plot():
    """Update the matplotlib plot with current grid data."""
    global fig, ax, im, current_grid_data

    if current_grid_data is None:
        return

    grid_array, width, height = current_grid_data

    if fig is None:
        # Initialize plot on first update
        fig, ax = plt.subplots(figsize=(12, 10))
        im = ax.imshow(
            grid_array,
            cmap=cmap,
            vmin=0,
            vmax=2,
            origin="lower",
            interpolation="nearest",
        )
        _ = ax.set_title("Pathfinding Grid Map", fontsize=14, fontweight="bold")
        _ = ax.set_xlabel("Width (cells)")
        _ = ax.set_ylabel("Height (cells)")

        # Add colorbar with labels
        cbar = plt.colorbar(im, ax=ax, ticks=[0, 1, 2])
        cbar.set_ticklabels(["Passable", "Obstacle", "Temporary"])
        cbar.set_label("Cell Type", rotation=270, labelpad=15)

        plt.tight_layout()
        _ = plt.ion()  # Turn on interactive mode
        plt.show()
    else:
        # Update existing plot
        if im is not None and ax is not None:
            im.set_data(grid_array)
            _ = ax.set_title(
                f"Pathfinding Grid Map ({width}x{height})",
                fontsize=14,
                fontweight="bold",
            )
            if fig is not None:
                fig.canvas.draw()
                fig.canvas.flush_events()


async def handle_grid_update(message: bytes):
    """Handle incoming grid update messages."""
    global current_grid_data

    try:
        grid = Grid.FromString(message)

        # Check which grid type we received
        which_grid = grid.WhichOneof("grid")

        if which_grid == "grid_2d":
            grid_2d = grid.grid_2d
            width = grid_2d.width
            height = grid_2d.height

            print(f"[GRID_UPDATE] Received Grid2d: {width}x{height}")

            # Convert flat array to 2D numpy array
            # Grid is stored row-major: [row0_col0, row0_col1, ..., row1_col0, ...]
            grid_array = np.array(grid_2d.grid, dtype=np.uint8)
            grid_array = grid_array.reshape((height, width))

            # Count cell types
            passable = sum(1 for cell in grid_2d.grid if cell == Cell.PASSABLE)
            obstacle = sum(1 for cell in grid_2d.grid if cell == Cell.OBSTACLE)
            temporary = sum(1 for cell in grid_2d.grid if cell == Cell.TEMPORARY)

            print(
                f"  Passable: {passable}, Obstacle: {obstacle}, Temporary: {temporary}"
            )

            # Update global grid data
            async with lock:
                current_grid_data = (grid_array, width, height)

            # Update plot (run in thread-safe way)
            plt.pause(0.001)  # Allow matplotlib to process events
            update_plot()

        elif which_grid == "grid_3d":
            grid_3d = grid.grid_3d
            width = grid_3d.width
            height = grid_3d.height
            depth = grid_3d.depth

            print(f"[GRID_UPDATE] Received Grid3d: {width}x{height}x{depth}")
            print("  Note: 3D grids not yet visualized, showing middle slice")

            # Convert flat array to 3D numpy array
            grid_array = np.array(grid_3d.grid, dtype=np.uint8)
            grid_array = grid_array.reshape((depth, height, width))

            # Show middle slice for 3D grids
            middle_slice = grid_array[depth // 2, :, :]

            # Count cell types
            passable = sum(1 for cell in grid_3d.grid if cell == Cell.PASSABLE)
            obstacle = sum(1 for cell in grid_3d.grid if cell == Cell.OBSTACLE)
            temporary = sum(1 for cell in grid_3d.grid if cell == Cell.TEMPORARY)

            print(
                f"  Passable: {passable}, Obstacle: {obstacle}, Temporary: {temporary}"
            )

            # Update global grid data with middle slice
            async with lock:
                current_grid_data = (middle_slice, width, height)

            # Update plot
            plt.pause(0.001)
            update_plot()
        else:
            print(f"[GRID_UPDATE] Unknown grid type: {which_grid}")

    except Exception as e:
        print(f"[GRID_UPDATE] Error processing grid message: {e}")
        import traceback

        traceback.print_exc()


async def main():
    client = Autobahn(Address("pinewood.local", 8080))
    await client.begin()

    print("[GRID_VIZ] Subscribing to pathfinding_map topic...")
    await client.subscribe("pathfinding_map", handle_grid_update)

    print(
        "[GRID_VIZ] Listening for grid updates. Close the plot window or Press Ctrl+C to stop."
    )

    # Keep the script running and update plot periodically
    try:
        while True:
            await asyncio.sleep(0.1)  # Small sleep to allow plot updates
            if fig is not None:
                plt.pause(0.01)  # Allow matplotlib to process events
    except KeyboardInterrupt:
        print("\n[GRID_VIZ] Shutting down...")
    finally:
        if fig is not None:
            plt.close("all")


if __name__ == "__main__":
    asyncio.run(main())
