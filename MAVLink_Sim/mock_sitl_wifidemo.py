# MAVLink Ports and System IDs
IM_RECEIVE_PORT = 14550      # Port where the IM Daemon listens for FC data
UAV_IP = '192.168.0.10'      # IP of the machine running the IM Daemon/GCS
UAV_IP = UAV_IP.strip()

SEND_RATE_HZ = 2             # Telemetry send rate (2Hz, or every 0.5s)
SYSTEM_ID_FC = 1
COMPONENT_ID_FC = 1

# Configuration for external network check
CHECK_HOST = "8.8.8.8"       # Google DNS server
CHECK_PORT = 53

import socket
import time
from pymavlink import mavutil
import math

def check_external_wifi_status(host=CHECK_HOST, port=CHECK_PORT, timeout=1):
    """Checks internet connectivity by attempting to connect to a known external host."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        return True
    except socket.error:
        return False
    finally:
        sock.close()

def mock_sitl_sender():
    """Simulates the Flight Controller sending HEARTBEAT and GLOBAL_POSITION_INT telemetry."""
    
    mav_encoder = mavutil.mavlink.MAVLink(SYSTEM_ID_FC, COMPONENT_ID_FC, use_native=False)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start_time = time.time()
    
    # Initial state simulation
    altitude = 10.0
    ground_speed = 5.0
    heading = 90.0 # East
    
    # Initial coordinates (multiplied by 1e7 as per GLOBAL_POSITION_INT spec)
    latitude = 340000000
    longitude = -1180000000
    
    last_check_time = 0.0
    
    print(f"Mock SITL started. Sending MAVLink packets to {UAV_IP}:{IM_RECEIVE_PORT}...")
    
    try:
        while True:
            current_time = time.time()
            current_time_ms = int((current_time - start_time) * 1000)
            
            # External Connectivity Check (Run every 5 seconds)
            if current_time - last_check_time > 5.0:
                is_connected = check_external_wifi_status()
                status = "✅ CONNECTED" if is_connected else "❌ DISCONNECTED"
                print(f"[{time.strftime('%H:%M:%S')}] Sender's Internet: {status} (Continuing MAVLink stream)")
                last_check_time = current_time
            
            # 1. Send HEARTBEAT (Used for initial link detection)
            heartbeat_msg = mav_encoder.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                int(1), 
                mavutil.mavlink.MAV_STATE_ACTIVE
            ).pack(mav_encoder)
            send_sock.sendto(heartbeat_msg, (UAV_IP, IM_RECEIVE_PORT))
            
            # 2. Send GLOBAL_POSITION_INT (Simulate slight, continuous movement)
            altitude += 0.05 
            step_size_e7 = 0.00005 * 1e7 # Movement step size
            
            # Update lat/lon based on heading
            latitude += int(step_size_e7 * math.cos(math.radians(heading))) 
            longitude += int(step_size_e7 * math.sin(math.radians(heading)))
            
            global_pos_msg = mav_encoder.global_position_int_encode(
                current_time_ms, 
                latitude, 
                longitude,
                int(altitude * 1000), # Alt MSL (mm)
                int(altitude * 1000), # Alt relative (mm) 
                int(ground_speed * 100), # VX (cm/s)
                int(ground_speed * 100), # VY (cm/s)
                int(0), # VZ (cm/s)
                int(heading * 100) # Heading (cdeg)
            ).pack(mav_encoder)
            send_sock.sendto(global_pos_msg, (UAV_IP, IM_RECEIVE_PORT))
            
            time.sleep(1.0 / SEND_RATE_HZ)

    except KeyboardInterrupt:
        print("\nMock SITL stopped.")
    finally:
        send_sock.close()

if __name__ == '__main__':
    mock_sitl_sender()
