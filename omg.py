# Define the PY_SSIZE_T_CLEAN macro
#define PY_SSIZE_T_CLEAN

import bluetooth

# The MAC address of your ESP32 (replace with your ESP32's MAC address)
esp32_mac_address = 'b4:8a:0a:75:e7:ee'

# Establish a Bluetooth connection
try:
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((esp32_mac_address, 1))
    print(f"Connected to ESP32 with MAC address {esp32_mac_address}")

    # Continuously listen for data from ESP32
    while True:
        data = sock.recv(1024)  # Receive data (adjust buffer size as needed)
        if not data:
            break  # Break the loop if no data received
        print(f"Received data: {data.decode()}")  # Decode and print the received data

    # Close the Bluetooth connection
    sock.close()
    print("Bluetooth connection closed")
except Exception as e:
    print(f"Error: {str(e)}")
