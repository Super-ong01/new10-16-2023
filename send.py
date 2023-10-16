# Define the PY_SSIZE_T_CLEAN macro
#define PY_SSIZE_T_CLEAN

import bluetooth

# The MAC address of your ESP32 (replace with your ESP32's MAC address)
esp32_mac_address = 'b4:8a:0a:75:e7:ee'
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

import cv2
import numpy as np

# Load YOLO model configuration and weights
model_config = r'yolov3-tiny.cfg'  # Path to YOLOv3 model configuration file
model_weights = r'yolov3-tiny.weights'  # Path to YOLOv3 model weights file

# Load YOLO class labels
with open(r'coco (1).names', 'r') as f:
    classes = f.read().strip().split('\n')

# Load the pre-trained YOLO model
net = cv2.dnn.readNet(model_weights, model_config)

# Set the preferred backend and target for OpenCV's DNN module
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  # You can use DNN_TARGET_CUDA for GPU acceleration if available

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    height, width = frame.shape[:2]

    # Create a blob from the frame and perform a forward pass
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    output_layers = net.getUnconnectedOutLayersNames()
    outputs = net.forward(output_layers)

    # Process detection results for elephants
    conf_threshold = 0.5
    nms_threshold = 0.4

    class_ids = []
    confidences = []
    boxes = []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > conf_threshold and classes[class_id] == 'person':
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

                # Print "gogogo" when an elephant is detected
                print("gogogo")

    # Apply Non-Maximum Suppression to remove overlapping detections
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    # Draw bounding boxes and labels on the frame for elephants
    for i in indices:
        
        box = boxes[i]
        x, y, w, h = box
        label = f'Elephant: {confidences[i]:.2f}'  # Construct the label
        color = (0, 255, 0)  # BGR color (green)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Display the frame with detections
    cv2.imshow('Elephant Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
