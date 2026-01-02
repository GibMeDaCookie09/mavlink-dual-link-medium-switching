# CONFIGURATION
IM_RECEIVE_PORT = 14550      # Port where the IM Daemon listens for FC data
UAV_IP = '127.0.0.1'         # Loopback IP
SEND_RATE_HZ = 2             # Telemetry send rate (2Hz, or every 0.5s)

# System and Component IDs for the FC
SYSTEM_ID_FC = 1
COMPONENT_ID_FC = 1

# CONTROLLED MOVEMENT CONFIG
PHASE_DURATION_SEC = 20.0    # Time in seconds for each movement phase (Away/Closer)
TRAVEL_SPEED_MPS = 20.0       # Controlled ground speed (m/s)
# Initial position (Home Base)
HOME_LAT = 340000000
HOME_LON = -1180000000

# IMPORTS AND SETUP
import socket
import time
from pymavlink import mavutil
import math
import random 

def mock_sitl_sender():
    """Simulates the Flight Controller sending HEARTBEAT and GLOBAL_POSITION_INT telemetry
    with controlled movement to cycle distance from home.
    """
    
    # Initialize MAVLink encoder for the FC. 
    mav_encoder = mavutil.mavlink.MAVLink(SYSTEM_ID_FC, COMPONENT_ID_FC, use_native=False)
    
    # Setup UDP socket to send data to the IM Daemon
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Record start time for time_boot_ms calculation
    start_time = time.time()
    
    # Initial state values
    altitude = 10.0
    
    # Geographic location (starting at Home Base)
    latitude = HOME_LAT
    longitude = HOME_LON
    
    # --- PHASE CONTROL STATE ---
    # Phases: 1=Moving Away (East, 90 deg), 2=Moving Closer (West, 270 deg)
    travel_phase = 1
    phase_start_time = start_time
    
    print(f"Mock SITL started. Sending MAVLink packets to {UAV_IP}:{IM_RECEIVE_PORT}...")
    print(f"Movement cycle: {PHASE_DURATION_SEC}s away, {PHASE_DURATION_SEC}s closer.")

    try:
        while True:
            current_time = time.time()
            current_time_ms = int((current_time - start_time) * 1000)
            
            # --- MOVEMENT PHASE LOGIC ---
            elapsed_phase_time = current_time - phase_start_time
            
            if elapsed_phase_time >= PHASE_DURATION_SEC:
                # Time to switch phase
                travel_phase += 1
                
                if travel_phase > 2:
                    # Reset cycle: Force back to home coordinates to simulate a new flight
                    travel_phase = 1
                    latitude = HOME_LAT
                    longitude = HOME_LON
                    print("\n[INFO] Cycle Reset: Position forced back to HOME.")
                
                phase_start_time = current_time
            
            # Set movement based on phase
            if travel_phase == 1:
                # Phase 1: Moving away (East - increasing distance)
                heading = 90.0
                ground_speed = TRAVEL_SPEED_MPS
            else: # travel_phase == 2
                # Phase 2: Moving closer (West - decreasing distance)
                heading = 270.0
                ground_speed = TRAVEL_SPEED_MPS

            # 1. Send HEARTBEAT (Critical C2)
            heartbeat_msg = mav_encoder.heartbeat_encode(
                int(mavutil.mavlink.MAV_TYPE_QUADROTOR),
                int(mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA),
                int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                int(0), # Setting custom_mode to 0 for compatibility
                int(mavutil.mavlink.MAV_STATE_ACTIVE)
            ).pack(mav_encoder)
            send_sock.sendto(heartbeat_msg, (UAV_IP, IM_RECEIVE_PORT))
            
            # 2. Send GLOBAL_POSITION_INT (Telemetry/Simulated Distance) 
            
            # Altitude increases slightly (simulated takeoff/climb)
            altitude += 0.05
            
            # Calculate movement step size (Distance = Speed * Time_per_cycle)
            time_step = 1.0 / SEND_RATE_HZ
            distance_step_m = ground_speed * time_step
            # Conversion factor: approx 1 degree lat = 111,000 meters. 1e7 is required for MAVLink
            step_size_e7 = (distance_step_m / 111000.0) * 1e7
            
            # Update position based on current heading
            # Note: Simple lat/lon movement approximation for simulation
            latitude += int(step_size_e7 * math.cos(math.radians(heading)))
            longitude += int(step_size_e7 * math.sin(math.radians(heading)))
            
            global_pos_msg = mav_encoder.global_position_int_encode(
                current_time_ms, 
                latitude,
                longitude,
                int(altitude * 1000),  # Alt MSL (mm)
                int(altitude * 1000),  # Alt relative (mm) 
                int(ground_speed * 100), # VX (cm/s)
                int(ground_speed * 100), # VY (cm/s)
                int(0), # VZ (cm/s)
                int(heading * 100) # Heading (cdeg)
            ).pack(mav_encoder)
            send_sock.sendto(global_pos_msg, (UAV_IP, IM_RECEIVE_PORT))
            
            # Wait to maintain send rate
            time.sleep(time_step)

    except KeyboardInterrupt:
        print("\nMock SITL stopped.")
    finally:
        send_sock.close()

if __name__ == '__main__':
    mock_sitl_sender()
