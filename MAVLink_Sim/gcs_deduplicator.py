import socket
import time
from pymavlink import mavutil
import select
import os

# CONFIGURATION

WIFI_RECEIVE_PORT = 14551  # Receives Wi-Fi data from IM Daemon
LORA_RECEIVE_PORT = 14552  # Receives LoRa data from IM Daemon
MP_OUTPUT_PORT = 14553     # Output port for Mission Planner connection
LISTEN_IP = '127.0.0.1'    # Retained as a constant, but we use '0.0.0.0' for listening
MP_IP = '127.0.0.1'

# Time window to detect duplicate packets (0.5 seconds)
DEDUPLICATION_WINDOW_S = 0.5 

# DEDUPLICATION LOGIC

class Deduplicator:
    def __init__(self, output_port):
        # Stores {packet_hash: timestamp} for tracking recently seen packets
        self.processed_hashes = {}
        self.output_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.output_addr = (MP_IP, output_port)
        self.duplicates_discarded = 0
        
        # MAVLink encoder for sending periodic GCS HEARTBEATs
        self.gcs_encoder = mavutil.mavlink.MAVLink(
            255, # SYSTEM_ID_GCS 
            1,   # COMPONENT_ID_GCS
            use_native=False
        )
        self.last_heartbeat_time = 0

    def send_heartbeat(self):
        """Sends a minimal GCS heartbeat to establish and maintain connection."""
        current_time = time.time()
        if current_time - self.last_heartbeat_time > 1.0: # Send every 1 second
            heartbeat_msg = self.gcs_encoder.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, # base_mode
                0, # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE
            ).pack(self.gcs_encoder)
            
            self.output_sock.sendto(heartbeat_msg, self.output_addr)
            self.last_heartbeat_time = current_time
        
    def process_and_forward(self, packet_bytes):
        """Checks for duplication and forwards unique packets to Mission Planner."""
        
        # Corrected for unhashable type error
        packet_hash = hash(bytes(packet_bytes))
        current_time = time.time()
        
        # 1. Clean up old hashes outside the time window
        self.processed_hashes = {
            h: t for h, t in self.processed_hashes.items() 
            if t > current_time - DEDUPLICATION_WINDOW_S
        }
        
        # 2. Check for Duplication
        if packet_hash in self.processed_hashes:
            self.duplicates_discarded += 1
            return False, "DUPLICATE"
        
        # 3. Process and Forward Unique Packet
        self.processed_hashes[packet_hash] = current_time
        self.output_sock.sendto(packet_bytes, self.output_addr)
        return True, "FORWARDED"

def gcs_deduplicator_main():
    
    deduplicator = Deduplicator(MP_OUTPUT_PORT)

    # 'udpin:0.0.0.0' to ensure we capture packets sent to the loopback IP
    wifi_conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{WIFI_RECEIVE_PORT}', timeout=0, planner_format=False)
    lora_conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{LORA_RECEIVE_PORT}', timeout=0, planner_format=False)
    
    connections = {'WIFI': wifi_conn, 'LORA': lora_conn}
    
    # Get the actual file descriptors for select.select()
    input_fds = [conn.port.fileno() for conn in connections.values()]

    print(f"GCS Agent running. Outputting clean stream to MP on port {MP_OUTPUT_PORT}.")
    print(f"Listening on WIFI port: {WIFI_RECEIVE_PORT} and LORA port: {LORA_RECEIVE_PORT}")
    
    try:
        while True:
            # Use select to wait for activity on the input ports with a small timeout
            # Note: We don't check `readable` against the conn.port.fileno() list 
            # because select.select only returns file descriptors, not objects.
            readable, _, _ = select.select(input_fds, [], [], 0.05)
            
            packets_received = 0
            
            # Iterate through all connections to process pending data
            for name, conn in connections.items():
                
                # Check if the connection's file descriptor is ready to be read
                if conn.port.fileno() in readable:
                    
                    # Process all available MAVLink messages from this socket
                    while True:
                        # Use recv_match() with no arguments to get the raw message buffer
                        msg = conn.recv_msg()
                        
                        if not msg:
                            break  # No more MAVLink packets waiting
                        
                        # Crucial check: ignore bad data messages
                        if msg.get_type() == 'BAD_DATA':
                            continue
                        
                        packets_received += 1
                        
                        # Get the raw MAVLink packet bytes
                        packet_bytes = msg.get_msgbuf()
                        
                        # Process, deduplicate, and forward
                        deduplicator.process_and_forward(packet_bytes)
            
            # Send periodic HEARTBEAT to ensure Mission Planner stays connected
            deduplicator.send_heartbeat()

            # Print status every time we receive data
            if packets_received > 0:
                print(f"Received {packets_received} packets. Deduplicated: {deduplicator.duplicates_discarded}")
                
    except KeyboardInterrupt:
        print("\nDeduplicator stopped.")
    except Exception as e:
        print(f"\nFATAL ERROR in Deduplicator: {e}")
    finally:
        for conn in connections.values():
            if conn:
                conn.close()
        deduplicator.output_sock.close()

if __name__ == '__main__':
    gcs_deduplicator_main()