import cv2
import mediapipe as mp
from pydobot import Dobot
from serial.tools import list_ports
import serial
import math  # Add this import for math functions

# Initialize MediaPipe hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

# Initialize Dobot
def find_dobot_port():
    ports = list_ports.comports()
    for port in ports:
        if 'Silicon Labs CP210x USB to UART Bridge' in port.description:
            return port.device
    return None

dobot_port = find_dobot_port()
if dobot_port is None:
    print("Dobot not found. Please check the connection and try again.")
else:
    try:
        device = Dobot(port=dobot_port)
        print(f"Dobot connected successfully on port {dobot_port}.")
    except Exception as e:
        print(f"Could not open port {dobot_port}: {e}")
        dobot_port = None

# Function to detect hand gestures
def detect_gesture(hand_landmarks, hand_label):
    thumb_extended = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y
    ring_finger_extended = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y
    pinky_extended = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y
    middle_finger_extended = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y
    forefinger_extended = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y
    all_fingers_extended = all([hand_landmarks.landmark[mp_hands.HandLandmark(i)].y < hand_landmarks.landmark[mp_hands.HandLandmark(i-1)].y for i in range(8, 21, 4)])
    
    if hand_label == 'Right':
        if thumb_extended and pinky_extended and forefinger_extended and not middle_finger_extended and not ring_finger_extended:
            return 'ARM UP'
        elif thumb_extended and not pinky_extended and not forefinger_extended and not middle_finger_extended and ring_finger_extended:
            return 'ARDUINO RIGHT'
        elif pinky_extended and thumb_extended and not forefinger_extended  and not middle_finger_extended and not ring_finger_extended :
            return 'ROTATE LEFT'
        elif forefinger_extended and thumb_extended and not pinky_extended and not middle_finger_extended and not ring_finger_extended:
            return 'AWAY'
        elif all_fingers_extended:
            return 'AIRPUMP ON'
    elif hand_label == 'Left':
        if thumb_extended and pinky_extended and forefinger_extended and not middle_finger_extended and not ring_finger_extended:
            return 'ARM DOWN'
        elif thumb_extended and not pinky_extended and not forefinger_extended and not middle_finger_extended and ring_finger_extended:
            return 'ARDUINO LEFT'
        elif  pinky_extended and thumb_extended and not forefinger_extended  and not middle_finger_extended and not ring_finger_extended :
            return 'ROTATE RIGHT'
        elif forefinger_extended and thumb_extended and not pinky_extended and not middle_finger_extended and not ring_finger_extended:
            return 'CLOSER'
        elif all_fingers_extended:
            return 'AIRPUMP OFF'
    return None

# Function to check if the Dobot is in a non-functional area
def check_non_functional_area(x, y, z):
    # Define the boundaries of the non-functional area (example values)
    non_functional_area = (x < 0 or x > 250 or y < -150 or y > 150 or z < 0 or z > 100)
    return non_functional_area

# Open the webcam
cap = cv2.VideoCapture(0)
if dobot_port:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Flip the frame horizontally for a selfie-view display
        frame = cv2.flip(frame, 1)
        
        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the image and detect hands
        results = hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            for hand_landmarks, hand_world_landmarks in zip(results.multi_hand_landmarks, results.multi_handedness):
                # Determine the hand label
                hand_label = hand_world_landmarks.classification[0].label
                
                # Draw hand landmarks on the frame
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Detect gesture
                gesture = detect_gesture(hand_landmarks, hand_label)
                
                if gesture:
                    # Control Dobot Magician based on gestures
                    pose = device.pose()
                    x, y, z, r, j1, j2, j3, j4 = pose
                    
                    if gesture == 'ARM UP':
                        device.move_to(x, y, z + 15, r, wait=False)
                    elif gesture == 'ARM DOWN':
                        device.move_to(x, y, z - 15, r, wait=False)
                    elif gesture == 'ARDUINO RIGHT':
                        pass # Send command to move stepper motor right
                    elif gesture == 'ARDUINO LEFT':
                        pass  # Send command to move stepper motor left
                    elif gesture == 'CLOSER':
                        # Move radially inward (toward the robot base)
                        angle = math.atan2(y, x)  # Current angle from base
                        new_x = x - 15 * math.cos(angle)
                        new_y = y - 15 * math.sin(angle)
                        device.move_to(new_x, new_y, z, r, wait=False)
                    elif gesture == 'AWAY':
                        # Move radially outward (away from the robot base)
                        angle = math.atan2(y, x)  # Current angle from base
                        new_x = x + 15 * math.cos(angle)
                        new_y = y + 15 * math.sin(angle)
                        device.move_to(new_x, new_y, z, r, wait=False)
                    elif gesture == 'ROTATE RIGHT':
                        # Move tangentially to create clockwise rotation around base
                        angle = math.atan2(y, x)  # Current angle from base
                        distance = math.sqrt(x**2 + y**2)  # Radial distance from base
                        new_angle = angle - math.radians(10)  # Rotate 10 degrees clockwise
                        new_x = distance * math.cos(new_angle)
                        new_y = distance * math.sin(new_angle)
                        device.move_to(new_x, new_y, z, r, wait=False)
                    elif gesture == 'ROTATE LEFT':
                        # Move tangentially to create counter-clockwise rotation around base
                        angle = math.atan2(y, x)  # Current angle from base
                        distance = math.sqrt(x**2 + y**2)  # Radial distance from base
                        new_angle = angle + math.radians(10)  # Rotate 10 degrees counter-clockwise
                        new_x = distance * math.cos(new_angle)
                        new_y = distance * math.sin(new_angle)
                        device.move_to(new_x, new_y, z, r, wait=False)
                    
                    # Display the gesture on the camera panel
                    cv2.putText(frame, gesture, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # Display the resulting frame
        cv2.imshow('Hand Gesture Recognition', frame)
        
        # End the code when 'q' key is pressed
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
if dobot_port:
    device.close()
