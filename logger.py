import serial
import time

SERIAL_PORT = ''  
BAUD_RATE = 9600
OUTPUT_FILE = "trees_log.txt"

print("Enter the serial port the Arduino is connected to")
SERIAL_PORT = input()
print(f"Connecting to Arduino on {SERIAL_PORT}...")
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Give Arduino time to reset/initialize
    print(f"Connected! Logging data to {OUTPUT_FILE}. Press Ctrl+C to stop.")
    
    with open(OUTPUT_FILE, "a") as file:
        while True:
            if arduino.in_waiting > 0:
                # Read line from Arduino and decode it
                line = arduino.readline().decode('utf-8').strip()
                
                if line:
                    # Generate a timestamp
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    
                    # Split the raw and percent values
                    try:
                        percent, temp, humidity = line.split(',')
                        log_entry = f"[{timestamp}] Moisture: {percent}% | Temp: {temp} | Humidity: {humidity}%\n"
                        
                        # Print to computer terminal and save to text file
                        print(log_entry.strip())
                        file.write(log_entry)
                        file.flush() # Ensure data writes immediately
                    except ValueError:
                        # Handles any fragmented serial data errors on startup
                        continue
                        
            time.sleep(0.1)

except serial.SerialException:
    print(f"Error: Could not open port {SERIAL_PORT}. Is the Arduino IDE Serial Monitor open? (Close it first!)")
except KeyboardInterrupt:
    print("\nLogging stopped successfully.")
