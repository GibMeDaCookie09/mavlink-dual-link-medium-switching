# analyze_log.py

import pandas as pd
import matplotlib.pyplot as plt

LOG_FILENAME = 'im_daemon_log.csv'

def analyze_and_plot_log():
    """Reads the log file, processes data, and generates a plot."""
    
    try:
        # Read the CSV file into a pandas DataFrame
        df = pd.read_csv(LOG_FILENAME)
    except FileNotFoundError:
        print(f"Error: Log file '{LOG_FILENAME}' not found. Please run mavlink_im_daemon.py first.")
        return
    except pd.errors.EmptyDataError:
        print("Error: Log file is empty. Run the daemon for a few seconds to generate data.")
        return

    print(f"Successfully loaded {len(df)} data points from {LOG_FILENAME}.")
    
    #  Data Processing for Visualization 
    
    # Map the ActiveLink string to a numerical value for plotting on a secondary y-axis
    link_mapping = {'Wi-Fi': 1.0, 'LoRa': 0.0}
    df['ActiveLink_Val'] = df['ActiveLink'].map(link_mapping)
    
    # Calculate Wi-Fi reliability (1 - Loss Rate)
    df['WiFi_Reliability'] = 1 - df['WiFi_Loss']
    df['LoRa_Reliability'] = 1 - df['LoRa_Loss']
    
    #  Plotting 
    
    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Plot 1: RSSI and Reliability vs. Distance (Primary Y-axis)
    color_rssi = 'tab:blue'
    ax1.set_xlabel('Distance from GCS (m)')
    ax1.set_ylabel('RSSI (dBm) / Reliability', color=color_rssi)
    
    ax1.plot(df['Distance_m'], df['RSSI'], color=color_rssi, label='RSSI', linestyle='-')
    ax1.tick_params(axis='y', labelcolor=color_rssi)
    
    # Plot reliability on the same axis (0 to 1 scale)
    ax1.plot(df['Distance_m'], df['WiFi_Reliability'], color='tab:green', linestyle='--', label='WiFi Reliability (1 - Loss)')
    ax1.plot(df['Distance_m'], df['LoRa_Reliability'], color='tab:orange', linestyle=':', label='LoRa Reliability (1 - Loss)')
    ax1.legend(loc='lower left')
    ax1.grid(True)
    
    # Create a secondary y-axis for the Active Link status
    ax2 = ax1.twinx()  
    color_link = 'tab:red'
    ax2.set_ylabel('Active Link Status', color=color_link)
    
    # Plot Active Link Status (1.0=Wi-Fi, 0.0=LoRa)
    ax2.plot(df['Distance_m'], df['ActiveLink_Val'], color=color_link, linestyle='none', marker='.', markersize=5, label='Active Link')
    ax2.tick_params(axis='y', labelcolor=color_link)
    ax2.set_yticks([0.0, 1.0])
    ax2.set_yticklabels(['LoRa Active', 'Wi-Fi Active'])
    ax2.set_ylim(-0.1, 1.1)

    plt.title('Interface Manager Link Performance vs. Distance')
    fig.tight_layout()
    plt.show()

if __name__ == '__main__':
    analyze_and_plot_log()