import serial
import serial.tools.list_ports
import time
import csv
from scipy.io import savemat

# Generate unique filenames for this session
SESSION_ID = time.strftime("%Y%m%d-%H%M%S")
CSV_FILENAME = f"ltr390_log_{SESSION_ID}.csv"
MAT_FILENAME = f"ltr390_log_{SESSION_ID}.mat"

def find_uno_q_port():
    """Scans all active COM ports and looks for the Arduino."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "USB Serial Device" in port.description:
            return port.device
    return None

def main():
    print("--- LTR390 Data Logging Console ---")
    print(f"Logging session started. Files will be saved as: \n- {CSV_FILENAME}\n- {MAT_FILENAME}")
    
    # Initialize data structures for MATLAB export
    mat_data = {
        'time_s': [],
        'uv_index': [],
        'uv_irradiance_W_m2': [],
        'illuminance_lux': [],
        'solar_irradiance_W_m2': []
    }

    start_time = time.time()
    ser = None

    # Open CSV in append mode and write the headers
    with open(CSV_FILENAME, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "Time (s)", 
            "UV Index", 
            "UV Irradiance (W/m^2)", 
            "Illuminance (Lux)", 
            "Estimated Solar Irradiance (W/m^2)"
        ])

        try:
            while True:
                try:
                    port = find_uno_q_port()

                    if not port:
                        print("Status: Waiting for Uno Q to be plugged in...", end='\r')
                        time.sleep(2)
                        continue

                    print(f"\n[+] Arduino found on {port}. Establishing connection...")
                    ser = serial.Serial(port, baudrate=115200, timeout=2)
                    time.sleep(2)
                    print("[+] Connection established. Logging data...\n")

                    while True:
                        if ser.in_waiting > 0:
                            raw_line = ser.readline().decode('utf-8').strip()

                            if not raw_line:
                                continue

                            if raw_line == "ERR|SENSOR_MISSING":
                                print("[!] CIRCUIT ALERT: Sensor not detected.")
                                continue

                            if raw_line.startswith("DATA|"):
                                parts = raw_line.split("|")
                                if len(parts) == 3:
                                    try:
                                        raw_uvs = int(parts[1])
                                        raw_als = int(parts[2])
                                        current_time = round(time.time() - start_time, 2)
                                        
                                        # 1. LTR390 Datasheet Conversions (For Gain=3, Resolution=18-bit)
                                        # Sensitivities are scaled down from the 20-bit/Gain=18 baseline in the datasheet
                                        uvi = round(raw_uvs / 95.83, 2)
                                        lux = round(raw_als * 0.2, 2)

                                        # 2. Engineering Radiometric Conversions
                                        uv_irradiance = round(uvi * 0.025, 4)       # 1 UVI = ~0.025 W/m^2
                                        solar_irradiance = round(lux * 0.0079, 4)   # Daylight approx: 1 Lux = ~0.0079 W/m^2

                                        # Print to console
                                        print(f"T: {current_time}s | UVI: {uvi} ({uv_irradiance} W/m^2) | Lux: {lux} ({solar_irradiance} W/m^2)")

                                        # Save to CSV instantly
                                        csv_writer.writerow([current_time, uvi, uv_irradiance, lux, solar_irradiance])
                                        csv_file.flush() # Force write to disk immediately

                                        # Store in memory for MATLAB export
                                        mat_data['time_s'].append(current_time)
                                        mat_data['uv_index'].append(uvi)
                                        mat_data['uv_irradiance_W_m2'].append(uv_irradiance)
                                        mat_data['illuminance_lux'].append(lux)
                                        mat_data['solar_irradiance_W_m2'].append(solar_irradiance)

                                    except ValueError:
                                        print(f"[?] Could not parse numerical data: {raw_line}")
                                else:
                                    print(f"[?] Malformed data received: {raw_line}")

                except serial.SerialException:
                    print("\n[-] USB connection lost. Returning to scanning mode...")
                    if ser and ser.is_open:
                        ser.close()
                    time.sleep(2)

        except KeyboardInterrupt:
            print("\n\nTesting terminated by user.")
            if ser and ser.is_open:
                ser.close()
            
            # Write out the MATLAB file cleanly on exit
            if len(mat_data['time_s']) > 0:
                print(f"Saving MATLAB data structure to {MAT_FILENAME}...")
                savemat(MAT_FILENAME, mat_data)
                print("Save complete.")
            else:
                print("No data recorded; skipping MATLAB export.")

if __name__ == "__main__":
    main()
