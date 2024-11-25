import serial
import os
from datetime import datetime

# Configuration
COM_PORT = 'COM5'  # Replace with your Arduino's COM port
BAUD_RATE = 9600
BASE_FOLDER = "gps_data"

def create_unique_folder():
    # Create a unique folder based on the current timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder_name = os.path.join(BASE_FOLDER, timestamp)
    os.makedirs(folder_name, exist_ok=True)
    return folder_name

def main():
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
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(line)  # Show data on the console
                    file.write(line + "\n")  # Save to file
        except KeyboardInterrupt:
            print("\nRecording stopped by user.")
        finally:
            ser.close()
            print(f"Data saved to {file_path}")

if __name__ == "__main__":
    main()
