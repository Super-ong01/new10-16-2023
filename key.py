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

    # Continuously read keyboard input and send it to ESP32
    print("Type something and press Enter to send to ESP32 (Ctrl+C to exit):")
    while True:
        input_text = input()  # Read keyboard input
        sock.send(input_text.encode())  # Encode and send the input text

    # Close the Bluetooth connection
    sock.close()
    print("Bluetooth connection closed")
except KeyboardInterrupt:
    print("Keyboard interrupt. Exiting.")
except Exception as e:
    print(f"Error: {str(e)}")
