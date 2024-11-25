import serial
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuration
COM_PORT = 'COM5'  # Replace with your Arduino's COM port
BAUD_RATE = 9600
BASE_FOLDER = "gps_data"

# Initialize data storage
raw_lat, raw_lon, raw_alt = [], [], []
avg_lat, avg_lon, avg_alt = [], [], []

# Create a unique folder for saving data
def create_unique_folder():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder_name = os.path.join(BASE_FOLDER, timestamp)
    os.makedirs(folder_name, exist_ok=True)
    return folder_name

# Update plot dynamically
def update_plot(frame):
    plt.cla()  # Clear the axes

    # Plot Latitude
    plt.subplot(3, 1, 1)
    plt.plot(raw_lat, label="Raw Latitude", alpha=0.7)
    plt.plot(avg_lat, label="Avg Latitude", linestyle="--")
    plt.legend(loc="upper left")
    plt.title("Latitude")
    plt.xlabel("Samples")
    plt.ylabel("Latitude")

    # Plot Longitude
    plt.subplot(3, 1, 2)
    plt.plot(raw_lon, label="Raw Longitude", alpha=0.7)
    plt.plot(avg_lon, label="Avg Longitude", linestyle="--")
    plt.legend(loc="upper left")
    plt.title("Longitude")
    plt.xlabel("Samples")
    plt.ylabel("Longitude")

    # Plot Altitude
    plt.subplot(3, 1, 3)
    plt.plot(raw_alt, label="Raw Altitude", alpha=0.7)
    plt.plot(avg_alt, label="Avg Altitude", linestyle="--")
    plt.legend(loc="upper left")
    plt.title("Altitude")
    plt.xlabel("Samples")
    plt.ylabel("Altitude")

    plt.tight_layout()

def main():
    global raw_lat, raw_lon, raw_alt, avg_lat, avg_lon, avg_alt

    # Set up Serial connection
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")
    
    # Create a unique folder and file
    folder = create_unique_folder()
    file_path = os.path.join(folder, "gps_data.csv")

    with open(file_path, 'w') as file:
        print(f"Recording data to {file_path}")
        file.write("Raw Latitude,Raw Longitude,Raw Altitude,Avg Latitude,Avg Longitude,Avg Altitude\n")  # CSV Header

        try:
            # Start live plotting
            fig = plt.figure(figsize=(10, 8))
            ani = FuncAnimation(fig, update_plot, interval=500, cache_frame_data=False)
            plt.ion()
            plt.show()

            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Skip header or invalid lines
                    if "Raw Latitude" in line:
                        continue

                    print(line)
                    file.write(line + "\n")  # Save to file

                    # Parse the line into values
                    values = line.split(",")
                    if len(values) == 6:
                        try:
                            raw_lat.append(float(values[0]))
                            raw_lon.append(float(values[1]))
                            raw_alt.append(float(values[2]))
                            avg_lat.append(float(values[3]))
                            avg_lon.append(float(values[4]))
                            avg_alt.append(float(values[5]))
                        except ValueError:
                            print(f"Skipping invalid data line: {line}")
        except KeyboardInterrupt:
            print("\nRecording stopped by user.")
        finally:
            ser.close()
            plt.ioff()
            print(f"Data saved to {file_path}")

if __name__ == "__main__":
    main()

