# CONFIGURATION (MAVLink Ports and System IDs)
FC_RECEIVE_PORT = 14550      # Port where IM Daemon listens for the Mock FC
WIFI_FORWARD_PORT = 14551    # Port for Wi-Fi output 
LORA_FORWARD_PORT = 14552    # Port for LoRa output 

# The IP of the machine running the Ground Control Station (GCS)
UAV_IP = '192.168.1.5'       # IMPORTANT: Should match the GCS machine IP

SYSTEM_ID_FC = 1             
SYSTEM_ID_IM = 100           
SYSTEM_ID_GCS = 255          

# Link constants
LORA_FIXED_LOSS_RATE = 0.05
WIFI_SWITCH_THRESHOLD_RSSI = -80.0
WIFI_MAX_RSSI = -50.0
RSSI_HYSTERESIS_DBM = 5.0 # Hysteresis added to prevent rapid switching near threshold

# IMPORTS AND SETUP
import time
import socket
import select
import math
from pymavlink import mavutil
import random  
import sys     
import os      
import threading # For command input

# GLOBAL CONTROL FLAG FOR MANUAL TEST 
WIFI_IS_UP = True 

def user_input_thread():
    """Waits for user input to simulate Wi-Fi failure/recovery."""
    global WIFI_IS_UP
    
    time.sleep(1) 
    print("\n\n[COMMAND] Type 'down' to simulate Wi-Fi failure, 'up' to recover. (Press Enter after typing)")
    
    while True:
        try:
            # sys.stdin.readline is safer than input() in a multithreaded loop
            cmd = sys.stdin.readline().strip().lower() 
            if cmd == 'down':
                WIFI_IS_UP = False
                print(f"\n[STATUS @ {time.time():.2f}] 🚨 WIFI LINK FORCED DOWN (100% Loss). System should switch to LORA.")
            elif cmd == 'up':
                WIFI_IS_UP = True
                print(f"\n[STATUS @ {time.time():.2f}] ✅ WIFI LINK FORCED UP (1% Loss). System switches back to automatic/RSSI control.")
            elif cmd:
                print(f"\n[COMMAND] Invalid command '{cmd}'. Use 'up' or 'down'.")
        except EOFError:
            break
        except Exception as e:
            # Ignore common multithreaded input errors
            pass

class LinkChannel:

    def __init__(self, name, forward_port, loss_func):
        self.name = name
        self.forward_port = forward_port
        self.loss_func = loss_func

    def calculate_rssi(self, distance_m):
        """Simplified linear decay model for visualization."""
        if distance_m < 50:
            return WIFI_MAX_RSSI
        distance_ratio = min(distance_m / 400.0, 1.0) 
        rssi = WIFI_MAX_RSSI + ((-100.0 - WIFI_MAX_RSSI) * distance_ratio)
        return rssi

    def get_metrics(self, distance_m):
        """Calculates RSSI and Loss rate, using global flag for Wi-Fi failure test."""
        
        rssi = self.calculate_rssi(distance_m)
        
        if self.name == 'Wi-Fi':
            # Use manual control flag for failover test
            global WIFI_IS_UP
            if WIFI_IS_UP:
                loss_rate = 0.01  # Normal behavior (low loss)
            else:
                loss_rate = 1.0    # Simulated failure (100% loss)
        else: # LoRa
            loss_rate = LORA_FIXED_LOSS_RATE

        return rssi, max(0.0, min(1.0, loss_rate))

class InterfaceManager:
    """Manages link switching and packet forwarding based on simulated metrics."""
    def __init__(self):
        self.active_link = 'Wi-Fi' 
        self.last_altitude = 10.0
        self.last_lat = 0
        self.last_lon = 0
        self.initial_lat = 0
        self.initial_lon = 0
        
        self.wifi = LinkChannel('Wi-Fi', WIFI_FORWARD_PORT, None)
        self.lora = LinkChannel('LoRa', LORA_FORWARD_PORT, None)

        self.wifi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lora_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def calculate_distance(self, lat_int, lon_int):
        """Calculates distance (m) from the initial takeoff point using Haversine formula."""
        
        if self.initial_lat == 0:
            self.initial_lat = lat_int
            self.initial_lon = lon_int
            return 0.0
        
        R = 6371000  
        lat1 = self.initial_lat / 1e7
        lon1 = self.initial_lon / 1e7
        lat2 = lat_int / 1e7
        lon2 = lon_int / 1e7
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance_m = R * c
        return distance_m

    def get_links_to_forward(self, distance_m, msg_id):
        """Determines the active link based on manual override, then RSSI/distance."""
        
        rssi, w_loss = self.wifi.get_metrics(distance_m) 

        # --- 1. Manual Override (Highest Priority) ---
        if w_loss >= 0.99:  # Manual 'down' command
            if self.active_link != 'LoRa':
                print(f"\n[SWITCH @ {time.time():.2f}] Link switched to LoRa due to Wi-Fi failure (Forced Loss).")
            self.active_link = 'LoRa'
        
        elif w_loss < 0.1:  # Manual 'up' command (Wi-Fi is functional, now check signal strength)
            
            # --- 2. Adaptive RSSI Check (Medium Priority) ---
            
            # Switch to LoRa if Wi-Fi RSSI drops below threshold
            if self.active_link == 'Wi-Fi' and rssi < WIFI_SWITCH_THRESHOLD_RSSI:
                print(f"\n[SWITCH @ {time.time():.2f}] Link switched to LoRa due to low RSSI ({rssi:.1f} dBm).")
                self.active_link = 'LoRa'
            
            # Switch back to Wi-Fi if RSSI recovers (using hysteresis)
            elif self.active_link == 'LoRa' and rssi > (WIFI_SWITCH_THRESHOLD_RSSI + RSSI_HYSTERESIS_DBM):
                print(f"\n[SWITCH @ {time.time():.2f}] Link switched back to Wi-Fi due to recovered RSSI ({rssi:.1f} dBm).")
                self.active_link = 'Wi-Fi'
            
            # If RSSI is between the threshold and (threshold + hysteresis), keep current link to avoid oscillation.
        
        # --- 3. Packet Forwarding Logic ---
        
        # Heartbeats are always duplicated to ensure fast link status detection by GCS
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
            return [self.wifi_sock, self.lora_sock], 'DUPLICATE'

        # GLOBAL_POSITION_INT is dropped on LoRa to conserve bandwidth
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if self.active_link == 'Wi-Fi':
                return [self.wifi_sock], 'SINGLE_WIFI'
            else:
                return [], 'DROPPED_BY_IM'
        
        # Default behavior: forward telemetry on the active link
        if self.active_link == 'Wi-Fi':
            return [self.wifi_sock], 'SINGLE_WIFI'
        else:
            return [self.lora_sock], 'SINGLE_LORA'

    def forward_packet(self, raw_packet, sock_list, debug_mode):
        """Sends the raw packet to the list of specified sockets, applying simulated loss."""
        
        forwarded_count = 0
        for sock in sock_list:
            if sock == self.wifi_sock:
                port = WIFI_FORWARD_PORT
                link = self.wifi
            elif sock == self.lora_sock:
                port = LORA_FORWARD_PORT
                link = self.lora
            else:
                continue

            distance_m = self.calculate_distance(self.last_lat, self.last_lon)
            _, loss_rate = link.get_metrics(distance_m)

            if random.random() > loss_rate:
                # Packet successfully transmitted
                sock.sendto(raw_packet, (UAV_IP, port))
                forwarded_count += 1
            elif debug_mode:
                # Packet dropped
                print(f"[{link.name}] LOST (Simulated Loss Rate: {loss_rate:.2f})")
                        
        return forwarded_count

def im_daemon_main():
    """Main loop for the Interface Manager Daemon."""
    
    # START COMMAND THREAD
    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()

    daemon_start_time = time.time()
    im = InterfaceManager()
    LOG_FILENAME = 'im_daemon_log.csv'
    
    print(f"IM Daemon listening on 0.0.0.0:{FC_RECEIVE_PORT}...")
    print(f"Forwarding Wi-Fi to {UAV_IP}:{WIFI_FORWARD_PORT}, LoRa to {UAV_IP}:{LORA_FORWARD_PORT}.")
    
    try:
        # Use 'udpin:0.0.0.0' to listen on ALL available network interfaces
        mav_in = mavutil.mavlink_connection(f'udpin:0.0.0.0:{FC_RECEIVE_PORT}', timeout=0.01)
    except Exception as e:
        print(f"Error establishing connection to FC Mock: {e}")
        return

    # LOGGING SETUP
    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    log_file = open(LOG_FILENAME, 'w')
    csv_header = "Timestamp,Distance_m,RSSI,ActiveLink,WiFi_Loss,LoRa_Loss,Mode\n"
    log_file.write(csv_header)

    print("\nTime  | Dist  | RSSI  | Active | W Loss | L Loss | Mode")
    print("-------------------------------------------------------")
    
    try:
        while True:
            msg = mav_in.recv_msg()
            
            if msg:
                # Filter out messages not originating from the Flight Controller Mock
                if msg.get_srcSystem() != SYSTEM_ID_FC:
                    continue
                    
                raw_packet = msg.get_msgbuf()
                distance_m = im.calculate_distance(im.last_lat, im.last_lon)

                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    # Update IM state with latest GPS data
                    im.last_lat = msg.lat
                    im.last_lon = msg.lon
                    im.last_altitude = msg.alt / 1000.0 
                    
                    # Get link metrics
                    rssi, w_loss_rate = im.wifi.get_metrics(distance_m)
                    _, l_loss_rate = im.lora.get_metrics(distance_m)
                    
                    # Determine target links and mode (where the main logic/fix was applied)
                    target_sockets, mode = im.get_links_to_forward(distance_m, msg.id) 

                    current_time = time.time()
                    elapsed_time = current_time - daemon_start_time 
                    
                    # LOGGING DATA (CSV)
                    csv_data = (
                        f"{current_time:.3f},{distance_m:.2f},{rssi:.2f},"
                        f"{im.active_link},{w_loss_rate:.2f},{l_loss_rate:.2f},{mode}\n"
                    )
                    log_file.write(csv_data)
                    log_file.flush() 
                    
                    # SEQUENTIAL CONSOLE VISUALIZATION
                    log_string = (
                        f"{elapsed_time:5.2f} | {distance_m:5.0f} | {rssi:5.0f} | "
                        f"{im.active_link:<6} | {w_loss_rate:5.2f} | {l_loss_rate:5.2f} | {mode:<10}"
                    )
                    
                    print(log_string) 
                    
                    im.forward_packet(raw_packet, target_sockets, debug_mode=False)

                elif msg.get_type() == 'HEARTBEAT':
                    # Heartbeats are handled separately for duplication logic
                    target_sockets, mode = im.get_links_to_forward(distance_m, msg.id)
                    im.forward_packet(raw_packet, target_sockets, debug_mode=False)
                    
                time.sleep(0.001) 

    finally:
        log_file.close() 
        
if __name__ == '__main__':
    try:
        im_daemon_main()
    except KeyboardInterrupt:
        print('\nIM Daemon stopped by user.') 
    except Exception as e:
        print(f"\nAn error occurred in the IM Daemon: {e}")
