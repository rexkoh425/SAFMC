import logging
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Enable logging (optional)
logging.basicConfig(level=logging.INFO)

# URI of the bridge drone (usually "radio://0/80/2M")
URI = "radio://0/80/2M"

def swarm_data_received(data):
    """Callback for receiving swarm data."""
    print(f"Received data: {data}")

def main():
    # Initialize the Swarm object
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Register a callback for incoming data
        scf.cf.appchannel.packet_received.add_callback(swarm_data_received)
        
        print("Waiting for swarm data... (Press Ctrl+C to stop)")
        try:
            while True:
                time.sleep(1)  # Keep running
        except KeyboardInterrupt:
            print("Stopping...")

if __name__ == "__main__":
    main()
