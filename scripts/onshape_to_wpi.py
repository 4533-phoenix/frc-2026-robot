# Copyright (c) 2026 FRC Team 4533 (Phoenix)
#
# Use of this source code is governed by a BSD
# license that can be found in the LICENSE file
# at the root directory of this project.

import argparse
from typing import List


def main() -> None:
    """
    Main entry point for the Onshape to WPILib coordinate converter.

    Parses X, Y, Z coordinates from Onshape (Y+ Forward, X+ Right) and
    converts them to WPILib NWU (X+ Forward, Y+ Left).
    """
    parser = argparse.ArgumentParser(
        description="Convert Onshape Translation (Y+ Forward, X+ Right) to WPILib NWU (X+ Forward, Y+ Left)."
    )

    # Input: X Y Z from Onshape
    parser.add_argument(
        "coords",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="The X Y Z coordinates from Onshape",
    )

    args: argparse.Namespace = parser.parse_args()

    # The Swap Logic
    os_coords: List[float] = args.coords
    os_x, os_y, os_z = os_coords

    wpi_x: float = os_y
    wpi_y: float = -os_x
    wpi_z: float = os_z

    print("\n--- WPILib Translation ---")
    print(f"X (Forward): {wpi_x:.4f}")
    print(f"Y (Left):    {wpi_y:.4f}")
    print(f"Z (Up):      {wpi_z:.4f}")
    print("--------------------------\n")

    # Java snippet for convenience
    print(f"Java: new Translation3d({wpi_x:.4f}, {wpi_y:.4f}, {wpi_z:.4f});")


if __name__ == "__main__":
    main()
