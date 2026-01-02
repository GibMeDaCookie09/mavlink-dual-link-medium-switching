
# CONFIGURATION (MAVLink Ports and System IDs)

FC_RECEIVE_PORT = 14550     # Port where IM Daemon listens for the Mock FC
WIFI_FORWARD_PORT = 14551   # Port for Wi-Fi output 
LORA_FORWARD_PORT = 14552   # Port for LoRa output 
# --- IMPORTANT: USE 127.0.0.1 FOR SIM OR YOUR HOTSPOT IP FOR LIVE TEST ---
UAV_IP = '192.168.0.10' 

SYSTEM_ID_FC = 1            
SYSTEM_ID_IM = 100          
SYSTEM_ID_GCS = 255         

# Link constants
LORA_FIXED_LOSS_RATE = 0.05
HEARTBEAT_TIMEOUT_S = 5.0  # Timeout for link loss detection (seconds)


# IMPORTS AND SETUP

import time
import socket
import random
import os
import threading
from pymavlink import mavutil


# STATUS DISPLAY THREAD

def status_display_thread(im, daemon_start_time):
    """
    Prints the current IM status every 1 second and announces link changes.
    Uses im.check_link_timeout() to determine if Wi-Fi has timed out.
    """
    time.sleep(1.0)
    print("\nTime  | Active | W Loss | L Loss | Mode")
    print("---------------------------------------")

    last_active_link = im.active_link

    while True:
        current_time = time.time()
        elapsed_time = current_time - daemon_start_time

        # Trigger a status evaluation (uses heartbeat time inside IM)
        im.get_links_to_forward(0, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT)

        w_loss_rate = im.wifi.get_metrics(0)[1]
        l_loss_rate = im.lora.get_metrics(0)[1]
        timeout_active = im.check_link_timeout()

        if timeout_active:
            im.active_link = 'LoRa'
            mode = 'TIMEOUT'
        else:
            im.active_link = 'Wi-Fi'
            mode = 'OK'

        # If link changed, print a one-time informative message with reason
        if im.active_link != last_active_link:
            # Decide a human-readable reason
            if im.active_link == 'LoRa':
                reason = 'Heartbeat timeout (no heartbeat from FC)'
            else:
                reason = 'Heartbeat received from FC (recovery)'
            print(f"[INFO] Switched to {im.active_link} link ({mode}) at {elapsed_time:5.2f}s - reason: {reason}")
            last_active_link = im.active_link

        log_string = (
            f"{elapsed_time:5.2f} | {im.active_link:<6} | {w_loss_rate:5.2f} | {l_loss_rate:5.2f} | {mode:<10}"
        )
        print(log_string)

        time.sleep(1.0)

# ==============================================================================
# LINK CHANNEL CLASS
# ==============================================================================
class LinkChannel:
    def __init__(self, name, forward_port, loss_func=None):
        self.name = name
        self.forward_port = forward_port
        self.loss_func = loss_func

    def get_metrics(self, distance_m):
        """Returns fixed metrics for simplicity."""
        rssi = -50.0
        if self.name == 'Wi-Fi':
            loss_rate = 0.01
        else:
            loss_rate = LORA_FIXED_LOSS_RATE
        return rssi, max(0.0, min(1.0, loss_rate))

# ==============================================================================
# INTERFACE MANAGER CLASS
# ==============================================================================
class InterfaceManager:
    def __init__(self):
        self.active_link = 'Wi-Fi'
        self.last_altitude = 0.0
        self.last_lat = 0
        self.last_lon = 0
        self.initial_lat = 0
        self.initial_lon = 0
        self.last_heartbeat_time = 0.0

        self.wifi = LinkChannel('Wi-Fi', WIFI_FORWARD_PORT)
        self.lora = LinkChannel('LoRa', LORA_FORWARD_PORT)

        # Sockets used to forward to the virtual UAV / ports
        self.wifi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lora_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def calculate_distance(self, lat_int, lon_int):
        """Simplified placeholder: returns 0 always."""
        return 0.0

    def check_link_timeout(self):
        """Returns True if the last heartbeat is older than HEARTBEAT_TIMEOUT_S."""
        if self.last_heartbeat_time == 0.0:
            # No heartbeat seen yet -> consider link alive (don't switch to LoRa until we've seen at least one)
            return False
        return (time.time() - self.last_heartbeat_time) > HEARTBEAT_TIMEOUT_S

    def get_links_to_forward(self, distance_m, msg_id):
        """
        Decide which sockets to forward to based on current heartbeat timeout.
        msg_id is used to decide duplication for HEARTBEAT messages.
        """
        # Decide active link from timeout status
        if self.check_link_timeout():
            self.active_link = 'LoRa'
        else:
            self.active_link = 'Wi-Fi'

        # Duplicate HEARTBEAT to both links
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
            return [self.wifi_sock, self.lora_sock], 'DUPLICATE'

        # For position updates, only forward on active link
        if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if self.active_link == 'Wi-Fi':
                return [self.wifi_sock], 'SINGLE_WIFI'
            else:
                return [self.lora_sock], 'SINGLE_LORA'

        # Default: forward on active link
        if self.active_link == 'Wi-Fi':
            return [self.wifi_sock], 'SINGLE_WIFI'
        else:
            return [self.lora_sock], 'SINGLE_LORA'

    def forward_packet(self, raw_packet, sock_list, debug_mode=False):
        """Simulate packet forwarding with simple loss model."""
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

            _, loss_rate = link.get_metrics(0)

            if random.random() > loss_rate:
                try:
                    sock.sendto(raw_packet, (UAV_IP, port))
                except Exception:
                    # If send fails (e.g., unreachable), silently ignore in sim
                    pass
                forwarded_count += 1
            elif debug_mode:
                print(f"[{link.name}] LOST (Simulated Loss Rate: {loss_rate:.2f})")

        return forwarded_count

# ==============================================================================
# MAIN DAEMON FUNCTION
# ==============================================================================
def im_daemon_main():
    """Main loop for the Interface Manager Daemon."""
    daemon_start_time = time.time()
    im = InterfaceManager()
    LOG_FILENAME = 'im_daemon_log.csv'

    # Start status/display thread
    status_thread = threading.Thread(target=status_display_thread, args=(im, daemon_start_time), daemon=True)
    status_thread.start()

    print(f"IM Daemon listening on 0.0.0.0:{FC_RECEIVE_PORT} (expecting FC system id {SYSTEM_ID_FC})...")
    print(f"Forwarding Wi-Fi -> {WIFI_FORWARD_PORT}, LoRa -> {LORA_FORWARD_PORT} (to {UAV_IP})")

    # Open MAVLink UDP input
    try:
        mav_in = mavutil.mavlink_connection(f'udpin:0.0.0.0:{FC_RECEIVE_PORT}', timeout=0.01)
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
        return

    # Logging setup
    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)
    log_file = open(LOG_FILENAME, 'w')
    csv_header = "Timestamp,ActiveLink,WiFi_Loss,LoRa_Loss,Mode\n"
    log_file.write(csv_header)

    try:
        last_check_time = time.time()
        while True:
            # Non-blocking receive (timeout set in connection)
            msg = mav_in.recv_msg()

            # Periodically evaluate timeout even if no messages arrive
            current_time = time.time()
            if current_time - last_check_time >= 1.0:
                # This will set im.active_link if a timeout condition is met
                im.get_links_to_forward(0, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT)
                last_check_time = current_time

            if msg:
                # Filter out messages not from the FC (important in multi-system tests)
                try:
                    src_sys = msg.get_srcSystem()
                except Exception:
                    src_sys = getattr(msg, 'srcSystem', None)

                if src_sys != SYSTEM_ID_FC:
                    # ignore messages not from our FC
                    continue

                # Get raw bytes for forwarding
                try:
                    raw_packet = msg.get_msgbuf()
                except Exception:
                    raw_packet = None

                # Safe-get message id (pymavlink may expose different attributes)
                try:
                    msg_id = msg.get_msgId()
                except Exception:
                    msg_id = getattr(msg, 'id', None)

                # Update state for position messages
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    im.last_lat = msg.lat
                    im.last_lon = msg.lon
                    im.last_altitude = msg.alt / 1000.0

                    target_sockets, mode = im.get_links_to_forward(im.calculate_distance(im.last_lat, im.last_lon), msg_id)

                    # Log CSV
                    now = time.time()
                    rssi, w_loss_rate = im.wifi.get_metrics(0)
                    _, l_loss_rate = im.lora.get_metrics(0)
                    csv_data = f"{now:.3f},{im.active_link},{w_loss_rate:.2f},{l_loss_rate:.2f},{mode}\n"
                    log_file.write(csv_data)
                    log_file.flush()

                    if raw_packet:
                        im.forward_packet(raw_packet, target_sockets, debug_mode=False)

                # Heartbeat handling: update last_heartbeat_time (this triggers recovery)
                elif msg.get_type() == 'HEARTBEAT':
                    # Update heartbeat timestamp (only for the FC)
                    im.last_heartbeat_time = time.time()

                    # After receiving heartbeat, immediately evaluate forwarding (and recovery)
                    target_sockets, mode = im.get_links_to_forward(0, msg_id)
                    if raw_packet:
                        im.forward_packet(raw_packet, target_sockets, debug_mode=False)

            # small sleep to avoid busy spin
            time.sleep(0.001)

    finally:
        log_file.close()

# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == '__main__':
    try:
        im_daemon_main()
    except KeyboardInterrupt:
        print('\nIM Daemon stopped by user.')
    except Exception as e:
        print(f"\nAn error occurred in the IM Daemon: {e}")
