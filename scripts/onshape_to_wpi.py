import argparse

def main():
    parser = argparse.ArgumentParser(
        description="Convert Onshape Translation (Y+ Forward, X+ Right) to WPILib NWU (X+ Forward, Y+ Left)."
    )
    
    # Input: X Y Z from Onshape
    parser.add_argument("coords", nargs=3, type=float, 
                        metavar=('X', 'Y', 'Z'),
                        help="The X Y Z coordinates from Onshape")

    args = parser.parse_args()

    # The Swap Logic
    os_x, os_y, os_z = args.coords
    
    wpi_x = os_y
    wpi_y = -os_x
    wpi_z = os_z

    print("\n--- WPILib Translation ---")
    print(f"X (Forward): {wpi_x:.4f}")
    print(f"Y (Left):    {wpi_y:.4f}")
    print(f"Z (Up):      {wpi_z:.4f}")
    print("--------------------------\n")
    
    # Java snippet for convenience
    print(f'Java: new Translation3d({wpi_x:.4f}, {wpi_y:.4f}, {wpi_z:.4f});')

if __name__ == "__main__":
    main()