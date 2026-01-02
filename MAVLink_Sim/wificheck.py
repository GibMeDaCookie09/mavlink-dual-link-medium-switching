import socket
import time

def check_wifi_status(host="8.8.8.8", port=53, timeout=3):
    """
    Checks internet connectivity by attempting to connect to a known external host.
    
    Args:
        host (str): IP address of the external server (e.g., Google DNS).
        port (int): Port to check (e.g., 53 for DNS).
        timeout (int): Seconds to wait for a connection attempt.
    """
    print(f"Checking connection to {host}:{port}...")
    try:
        # Create a socket object (AF_INET for IPv4, SOCK_STREAM for TCP)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        # Attempt to connect (this is faster and more reliable than a true ICMP ping)
        sock.connect((host, port))
        
        # If connect succeeds, the link is up
        return True

    except socket.error as e:
        # If connect fails (due to timeout, connection refused, or network unreachable)
        # the link is considered down.
        # print(f"Socket error details: {e}") # Optional: for debugging
        return False
    
    finally:
        sock.close()

if __name__ == "__main__":
    
    # You can change the host to any reliable external server (Google's is common)
    TARGET_HOST = "8.8.8.8" 
    
    print("--- Wi-Fi Disconnection Monitor Started ---")
    
    while True:
        is_connected = check_wifi_status(host=TARGET_HOST)
        
        if is_connected:
            print(f"[{time.strftime('%H:%M:%S')}] ✅ Wi-Fi Connected (Internet Reachable)")
        else:
            # If the script prints this line, your physical Wi-Fi connection is likely severed.
            print(f"[{time.strftime('%H:%M:%S')}] ❌ Wi-Fi Disconnected (Connection Failed)")
        
        time.sleep(5) # Check status every 5 seconds