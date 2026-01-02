
# CONFIGURATION (MAVLink Ports and System IDs)

FC_RECEIVE_PORT = 14550     # Port where IM Daemon listens for the Mock FC
WIFI_FORWARD_PORT = 14551   # Port for Wi-Fi output (listened to by Deduplicator)
LORA_FORWARD_PORT = 14552   # Port for LoRa output (listened to by Deduplicator)
UAV_IP = '127.0.0.1'        # Loopback IP
SYSTEM_ID_FC = 1            # ID of the Mock Flight Controller
SYSTEM_ID_IM = 100          # ID of this Companion Computer/IM
SYSTEM_ID_GCS = 255         # ID of the Ground Control Station

# Link thresholds and simulation constants
WIFI_SWITCH_THRESHOLD_RSSI = -60.0  # RSSI below this triggers switch to LoRa
LORA_FIXED_LOSS_RATE = 0.05         # Fixed 5% loss for robust LoRa
WIFI_MAX_RSSI = -50.0               # RSSI at closest range
TIME_TO_LIVE = 3                    # TTL for critical packets (used in duplication check)


# IMPORTS AND SETUP
import time
import socket
import select
import math
from pymavlink import mavutil
import random  
import sys     
import os      # Added for file operations

#  1. Link Models (Simulation Logic) 

class LinkChannel:
    """Simulates the physical characteristics of a communication link."""
    def __init__(self, name, forward_port, loss_func):
        self.name = name
        self.forward_port = forward_port
        self.loss_func = loss_func

    def calculate_rssi(self, distance_m):
        """Calculates RSSI based on simulated distance decay."""
        # Simple linear decay model (RSSI improves as distance decreases)
        if distance_m < 50:
            return WIFI_MAX_RSSI
        
        # Max distance simulated in mock_sitl.py is about 350m
        distance_ratio = min(distance_m / 400.0, 1.0) 
        rssi = WIFI_MAX_RSSI + ((-100.0 - WIFI_MAX_RSSI) * distance_ratio)
        return rssi

    def get_metrics(self, distance_m):
        """Calculates RSSI and Loss rate for the current simulated conditions."""
        
        rssi = self.calculate_rssi(distance_m)
        
        if self.name == 'Wi-Fi':
            # Wi-Fi: Excellent when close, rapidly degrades based on RSSI
            if rssi > -65: 
                loss_rate = 0.01 
            elif rssi < WIFI_SWITCH_THRESHOLD_RSSI: 
                loss_rate = 1.0  # Link broken
            else:
                # Linear degradation
                loss_rate = 0.01 + 0.99 * ((-65 - rssi) / (WIFI_SWITCH_THRESHOLD_RSSI - (-65)))
        else: # LoRa
            # LoRa: Stable, low loss regardless of typical distance
            loss_rate = LORA_FIXED_LOSS_RATE

        return rssi, max(0.0, min(1.0, loss_rate))

#  2. Interface Manager 

class InterfaceManager:
    """Handles link selection and duplication logic."""
    def __init__(self):
        self.active_link = 'Wi-Fi' # Start with high-throughput link
        self.last_altitude = 10.0
        self.last_lat = 0
        self.last_lon = 0
        self.initial_lat = 0
        self.initial_lon = 0
        
        # Define simulated channels
        self.wifi = LinkChannel('Wi-Fi', WIFI_FORWARD_PORT, None)
        self.lora = LinkChannel('LoRa', LORA_FORWARD_PORT, None)

        # Output sockets for forwarding duplicated packets
        self.wifi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lora_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def calculate_distance(self, lat_int, lon_int):
        """Calculates great-circle distance (Haversine) from initial position."""
        if self.initial_lat == 0:
            self.initial_lat = lat_int
            self.initial_lon = lon_int
            return 0.0

        # Convert MAVLink int (1e7) to degrees (float)
        lat1 = self.initial_lat / 1e7
        lon1 = self.initial_lon / 1e7
        lat2 = lat_int / 1e7
        lon2 = lon_int / 1e7

        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance_m = R * c

        return distance_m

    def get_links_to_forward(self, distance_m, msg_id):
        """Determines the active link and if duplication is necessary."""
        
        rssi, _ = self.wifi.get_metrics(distance_m)

        # 1. Dynamic Link Switching
        if rssi < WIFI_SWITCH_THRESHOLD_RSSI:
            # Force switch to LoRa (fallback)
            self.active_link = 'LoRa'
        elif rssi > (WIFI_SWITCH_THRESHOLD_RSSI + 5):
            # Switch back to Wi-Fi (hysteresis of 5 dBm)
            self.active_link = 'Wi-Fi'

        # 2. Duplication Logic
        # HEARTBEAT is critical C2/T, always duplicated for max reliability
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
            return [self.wifi_sock, self.lora_sock], 'DUPLICATE'

        # GLOBAL_POSITION_INT is high-rate telemetry, check active link
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if self.active_link == 'Wi-Fi':
                # Use high-throughput link
                return [self.wifi_sock], 'SINGLE_WIFI'
            else:
                # On LoRa, only forward the most essential C2/T (HEARTBEAT), drop high-rate T
                return [], 'DROPPED_BY_IM'
        
        # Default: just use the active link (e.g., if it were a COMMAND_ACK)
        if self.active_link == 'Wi-Fi':
            return [self.wifi_sock], 'SINGLE_WIFI'
        else:
            return [self.lora_sock], 'SINGLE_LORA'

    def forward_packet(self, raw_packet, sock_list, debug_mode):
        """Sends the raw packet over the list of destination sockets."""
        
        forwarded_count = 0
        for sock in sock_list:
            if sock == self.wifi_sock:
                port = WIFI_FORWARD_PORT
            elif sock == self.lora_sock:
                port = LORA_FORWARD_PORT
            else:
                continue

            #  3. Simulate Loss for Packet 
            link = self.wifi if port == WIFI_FORWARD_PORT else self.lora
            
            distance_m = self.calculate_distance(self.last_lat, self.last_lon)
            _, loss_rate = link.get_metrics(distance_m)

            if random.random() > loss_rate:
                # Packet successfully transmitted (not lost)
                sock.sendto(raw_packet, (UAV_IP, port))
                forwarded_count += 1
                if debug_mode:
                    print(f"[{link.name}] FORWARDED to {port} (Loss Rate: {loss_rate:.2f})")
            elif debug_mode:
                print(f"[{link.name}] LOST (Simulated Loss Rate: {loss_rate:.2f})")
                    
        return forwarded_count

def im_daemon_main():
    """Main loop for the Interface Manager Daemon."""
    
    # --- FIX 1: Record the daemon's start time ---
    daemon_start_time = time.time()

    im = InterfaceManager()
    
    # --- LOGGING CONFIGURATION ---
    LOG_FILENAME = 'im_daemon_log.csv'
    
    print(f"IM Daemon listening on {UAV_IP}:{FC_RECEIVE_PORT}...")
    print(f"Forwarding Wi-Fi to {WIFI_FORWARD_PORT}, LoRa to {LORA_FORWARD_PORT}.")
    
    try:
        mav_in = mavutil.mavlink_connection(f'udpin:{UAV_IP}:{FC_RECEIVE_PORT}', timeout=0.01)
    except Exception as e:
        print(f"Error establishing connection to FC Mock: {e}")
        return

    #  LOGGING SETUP 
    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
        
    log_file = open(LOG_FILENAME, 'w')
    
    csv_header = "Timestamp,Distance_m,RSSI,ActiveLink,WiFi_Loss,LoRa_Loss,Mode\n"
    log_file.write(csv_header)

    # Print the console header once
    print("\nTime  | Dist  | RSSI  | Active | W Loss | L Loss | Mode")
    print("-------------------------------------------------------")
    
    try:
        while True:
            msg = mav_in.recv_msg()
            
            if msg:
                if msg.get_srcSystem() != SYSTEM_ID_FC:
                    continue
                    
                raw_packet = msg.get_msgbuf()
                distance_m = im.calculate_distance(im.last_lat, im.last_lon)

                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    im.last_lat = msg.lat
                    im.last_lon = msg.lon
                    im.last_altitude = msg.alt / 1000.0 
                    
                    rssi, w_loss_rate = im.wifi.get_metrics(distance_m)
                    _, l_loss_rate = im.lora.get_metrics(distance_m)
                    
                    target_sockets, mode = im.get_links_to_forward(distance_m, msg.id) 

                    current_time = time.time()
                    
                    #  SEQUENTIAL CONSOLE VISUALIZATION 
                    # FIX 2: Calculate elapsed time for display
                    elapsed_time = current_time - daemon_start_time
                    
                    # LOGGING DATA (CSV) - Uses full current_time for uniqueness
                    csv_data = (
                        f"{current_time:.3f},{distance_m:.2f},{rssi:.2f},"
                        f"{im.active_link},{w_loss_rate:.2f},{l_loss_rate:.2f},{mode}\n"
                    )
                    log_file.write(csv_data)
                    log_file.flush() 
                    
                    # Display output - Uses elapsed_time
                    log_string = (
                        f"{elapsed_time:5.2f} | {distance_m:5.0f} | {rssi:5.0f} | "
                        f"{im.active_link:<6} | {w_loss_rate:5.2f} | {l_loss_rate:5.2f} | {mode:<10}"
                    )
                    
                    print(log_string) 
                    
                    im.forward_packet(raw_packet, target_sockets, debug_mode=False)

                elif msg.get_type() == 'HEARTBEAT':
                    target_sockets, mode = im.get_links_to_forward(distance_m, msg.id)
                    im.forward_packet(raw_packet, target_sockets, debug_mode=False)
                    
            time.sleep(0.001) 

    finally:
        log_file.close() 
        
if __name__ == '__main__':
    try:
        im_daemon_main()
    except KeyboardInterrupt:
        # Using print() here is fine as the console output is sequential
        print('\nIM Daemon stopped by user.') 
    except Exception as e:
        print(f"\nAn error occurred in the IM Daemon: {e}")