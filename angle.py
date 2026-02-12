import math

class LauncherController:
    # --- Constants ---
    A = 6.403  # Ground Link (Fixed)
    B = 7.521  # Crank Arm (Moving)
    
    C_MIN = 6.925
    C_MAX = 10.5
    
    DUTY_MIN = 100.0 # Corresponds to C_MIN (Most Vertical)
    DUTY_MAX = 200.0 # Corresponds to C_MAX (Most Horizontal)
    
    # --- Geometric Offsets ---
    # At C_MIN, Plate Angle (H) = 90.0
    # Internal Triangle Angle (Theta) for C_MIN is ~59.01
    # Because extending the servo (increasing Theta) DECREASES the Plate Angle:
    # Theta = Offset - Plate_Angle
    # 59.01 = Offset - 90.0  => Offset = 149.01
    GEOMETRIC_OFFSET = 149.01
    LAUNCH_OFFSET = 12.875 # Launch Angle = Plate Angle - 12.875

    @classmethod
    def get_duty_cycle(cls, target_launch_angle):
        """
        Input: 0 (Horizon) to 90 (Vertical)
        Output: 100 to 200 Duty Cycle
        """
        # 1. Clamp input to physical vertical limit
        # The mechanism stops at 90 deg plate angle, which is 77.125 deg launch
        max_possible_launch = 90.0 - cls.LAUNCH_OFFSET
        clamped_launch = min(target_launch_angle, max_possible_launch)
        clamped_launch = max(clamped_launch, 0.0) # Don't aim into the floor

        # 2. Convert Launch Angle to Plate Angle (H)
        plate_angle = clamped_launch + cls.LAUNCH_OFFSET
        
        # 3. Convert Plate Angle to Internal Triangle Angle (Theta)
        # Higher Theta = Lower Plate Angle
        theta_deg = cls.GEOMETRIC_OFFSET - plate_angle
        theta_rad = math.radians(theta_deg)

        # 4. Law of Cosines: Find required servo length (c)
        c_sq = (cls.A**2) + (cls.B**2) - (2 * cls.A * cls.B * math.cos(theta_rad))
        c_len = math.sqrt(max(0, c_sq))

        # 5. Map Length to Duty Cycle (100-200)
        # C_MIN (6.925) -> 100
        # C_MAX (10.5)  -> 200
        len_range = cls.C_MAX - cls.C_MIN
        duty_range = cls.DUTY_MAX - cls.DUTY_MIN
        
        # Normalize extension (0.0 to 1.0)
        extension_pct = (c_len - cls.C_MIN) / len_range
        extension_pct = max(0.0, min(1.0, extension_pct)) # Safety clamp

        duty_cycle = cls.DUTY_MIN + (extension_pct * duty_range)
        
        return duty_cycle, c_len

# --- Test Script ---
if __name__ == "__main__":
    # Test 77.125 (Max Up), 60 (Mid), 40 (Near Horizon)
    test_angles = [77.125, 60.0, 45.0, 38.5] 

    print(f"{'Launch Angle':<15} | {'Servo Length':<15} | {'Duty (100-200)':<15}")
    print("-" * 50)

    for angle in test_angles:
        duty, length = LauncherController.get_duty_cycle(angle)
        print(f"{angle:<15.2f} | {length:<15.3f} | {duty:<15.2f}")

    print("\nNote: At 77.125 deg launch, the plate is at 90 deg (Vertical).")
    print("Lowering the launch angle requires MORE extension (higher duty).")