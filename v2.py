import cv2
import numpy as np
import serial
import time

# Open serial to Teensy
ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Parameters
center_angle = 85          # straight wheels (matches Teensy)
turn_range = 30             # max degrees left/right from center

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    roi = frame[int(height*0.6):height, :]  # bottom half

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.GaussianBlur(mask, (5,5), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            error = (width // 2) - cx  # negative = line right, positive = line left

            # Proportional steering
            # Map error to turn_range
            steering_angle = center_angle + int((error / (width//2)) * turn_range)

            # Keep within safe range
            steering_angle = max(center_angle - turn_range, min(center_angle + turn_range, steering_angle))

            # Send to Teensy
            ser.write(f"{steering_angle}\n".encode())
            print("Steering angle:", steering_angle)

    else:
        # No line detected → go straight
        ser.write(f"{center_angle}\n".encode())
        print("No line detected, going straight")

    cv2.imshow("Mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
