import zenoh
import time

# CONFIGURATION
# ------------------------------------------------------------------
# If you are connected via WiFi/Ethernet and Multicast is working:
CONNECT_TO_ROBOT = True 

# If Multicast fails (common on FRC Radio), set this to True and 
# enter your RoboRIO IP (e.g., "10.TE.AM.2" or "172.22.11.2" for USB)
ROBOT_IP = "10.45.33.2" 
# ------------------------------------------------------------------

def listener(sample):
    """
    Callback function that triggers whenever a value is published.
    """
    # 'sample' contains the key (path) and the payload (value)
    key = sample.key_expr
    
    # In Zenoh 1.0+, payload is a ZBytes object. 
    # use .to_string() to get the text value.
    try:
        value = sample.payload.to_string()
    except:
        # Fallback for binary data
        value = f"<Binary Data: {len(sample.payload)} bytes>"

    print(f"[{key}] : {value}")

def main():
    # 1. Configure the session
    conf = zenoh.Config()
    
    if CONNECT_TO_ROBOT:
        # Directly connect to the RoboRIO if multicast is blocked
        print(f"[Info] Configuring direct connection to {ROBOT_IP}...")
        conf.insert_json5("connect/endpoints", f"['tcp/{ROBOT_IP}:7447']")
    else:
        print("[Info] Using auto-discovery (Multicast)...")

    # 2. Open the Zenoh session
    print("[Info] Opening Zenoh session...")
    with zenoh.open(conf) as session:
        
        # 3. Declare a subscriber
        # "frc/robot/**" matches ANYTHING starting with frc/robot/
        # This catches "frc/robot/battery", "frc/robot/pose", etc.
        print("[Info] Subscribing to 'frc/robot/**'...")
        sub = session.declare_subscriber("frc/robot/**", listener)

        print("\nListening for telemetry... Press CTRL+C to stop.\n")
        
        # 4. Keep the script running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Info] Closing session...")

if __name__ == "__main__":
    main()