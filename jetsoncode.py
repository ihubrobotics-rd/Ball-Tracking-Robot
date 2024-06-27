import serial
import time
import cv2
import numpy as np

defaultSpeed = 50
windowCenter = 320
centerBuffer = 10
pwmBound = float(50)
cameraBound = float(320)
kp = pwmBound / cameraBound
leftBound = int(windowCenter - centerBuffer)
rightBound = int(windowCenter + centerBuffer)
error = 0
ballPixel = 0

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

def updatePwm(rightPwm, leftPwm):
    command = f"MOTOR {rightPwm:03d} {leftPwm:03d}\n"
    arduino.write(command.encode())

def pwmStop():
    arduino.write("STOP\n".encode())

# Camera setup
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower_yellow = np.array([17, 98, 66])
upper_yellow = np.array([72, 255, 255])

while True:
    ret, frame = camera.read()
    if not ret:
        break
    
    output = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    output = cv2.bitwise_and(output, output, mask=mask)
    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 500, minRadius=10, maxRadius=200, param1=100, param2=60)
    
    ballPixel = 0
    radius = 0  # Initialize radius
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, radius) in circles:
            cv2.circle(output, (x, y), radius, (0, 255, 0), 4)
            if radius > 10:
                ballPixel = x
            else:
                ballPixel = 0
    
    key = cv2.waitKey(1) & 0xFF
    
    if ballPixel == 0:
        print("no ball")
        error = 0
        pwmStop()
    elif (ballPixel < leftBound) or (ballPixel > rightBound):
        error = windowCenter - ballPixel
        pwmOut = abs(error * kp)
        turnPwm = int(pwmOut + defaultSpeed)
        if ballPixel < leftBound:
            print("left side")
            if radius > 50 and ballPixel < 110:
                print(ballPixel)
                updatePwm(defaultSpeed, 20)
            else:
                updatePwm(turnPwm, defaultSpeed)
        elif ballPixel > rightBound:
            print("right side")
            if radius > 50 and ballPixel > 540:
                print(ballPixel)
                updatePwm(20, defaultSpeed)
            else:
                updatePwm(defaultSpeed, turnPwm)
    else:
        print("middle")
        if radius < 40:
            updatePwm(defaultSpeed, defaultSpeed)
        else:
            pwmStop()
    
    # Display the live video feed
    cv2.imshow("Live Video", output)
    
    if key == ord('q'):
        break

cv2.destroyAllWindows()
camera.release()
pwmStop()
arduino.close()
