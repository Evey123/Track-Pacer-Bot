import cv2
import numpy as np
import serial
import time

# Wait for Teensy
while True:
    try:
        ser = serial.Serial('/dev/ttyACM0',115200)
        break
    except:
        print("Waiting for Teensy...")
        time.sleep(1)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Parameters
center_angle = 85
turn_range = 30

# Motor parameters
neutral_throttle = 1500
forward_throttle = 1600

# Ignore small random detections
MIN_CONTOUR_AREA = 2000

while True:

    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    roi = frame[int(height*0.6):height, :]  # bottom part of image

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([20,60,60])
    upper_yellow = np.array([40,255,255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Blur
    mask = cv2.GaussianBlur(mask,(5,5),0)

    # Remove noise
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    try:

        if contours:

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > MIN_CONTOUR_AREA:

                M = cv2.moments(largest)

                if M["m00"] != 0:

                    cx = int(M["m10"] / M["m00"])
                    error = (width//2) - cx

                    steering_angle = center_angle + int((error/(width//2))*turn_range)

                    steering_angle = max(center_angle-turn_range,
                                         min(center_angle+turn_range, steering_angle))

                    ser.write(f"{steering_angle},{forward_throttle}\n".encode())

                    print(f"Steering: {steering_angle}  Throttle: {forward_throttle}  Area: {area}")

            else:

                ser.write(f"{center_angle},{neutral_throttle}\n".encode())
                print("Ignoring small detection")

        else:

            ser.write(f"{center_angle},{neutral_throttle}\n".encode())
            print("No line detected, stopping motor")

    except serial.SerialException:
        print("Serial connection lost. Waiting for Teensy...")
        while True:
            try:
                ser = serial.Serial('/dev/ttyACM0',115200)
                print("Reconnected.")
                break
            except:
                time.sleep(1)

    cv2.imshow("Mask", mask)
    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
