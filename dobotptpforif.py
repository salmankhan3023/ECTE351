import cv2
import mediapipe as mp
import time
from pydobotplus import Dobot
from serial.tools import list_ports
import numpy as np
import serial
import threading
import json

class DobotHandController:
    def __init__(self):
        # Constants and configuration
        self.MIN_DETECTION_CONFIDENCE = 0.7
        self.MIN_TRACKING_CONFIDENCE = 0.7
        self.COMMAND_DELAY = 0.5  # Reduced from 1.5s for more responsive movement
        
        # Safe operating ranges for Dobot
        self.DOBOT_X_RANGE = (100, 300)  # Min and max X values
        self.DOBOT_Y_RANGE = (-200, 200)  # Min and max Y values
        self.DOBOT_Z_RANGE = (-78, 140)   # Min and max Z values
        
        # Define home position and invalid positions
        self.HOME_POSITION = (200, 0, 0, 0)  # x, y, z, r
        self.invalid_zones = [
            # Define top right as invalid zone
            (200, 300, 150, 200, -78, 140),   # Top right invalid zone
            (200, 300, -200, -150, -78, 140)  # Top left invalid zone
        ]
        
        # State variables
        self.dobot_x, self.dobot_y, self.dobot_z, self.dobot_r = self.HOME_POSITION
        self.last_command_time = time.time()
        self.air_pump_on = False
        self.dobot = None
        self.dobot_connected = False
        self.in_invalid_zone = False
        self.returning_home = False
        self.error_reported = False
        
        # Arduino and motor control variables
        self.arduino = None
        self.arduino_connected = False
        self.ir_sensors_clear = True
        self.emergency_stop = False
        self.left_hand_detected = False
        self.motor_direction = "STOP"  # "LEFT", "RIGHT", "STOP"
        self.last_motor_command_time = time.time()
        self.motor_command_delay = 0.1  # More responsive for base movement
        
        # Left hand control zone parameters
        self.left_zone_width_ratio = 0.3  # 30% of screen width for left hand zone
        self.left_zone_boundary_threshold = 0.1  # 10% from edges to trigger movement
        
        # Initialize components
        self.setup_mediapipe()
        self.connect_arduino()
        self.connect_dobot()
        self.setup_camera()
        
        # Movement smoothing
        self.smoothing_factor = 0.3  # Lower = more smoothing
        self.target_x, self.target_y, self.target_z = self.dobot_x, self.dobot_y, self.dobot_z
        
        # Start Arduino monitoring thread
        self.start_arduino_monitoring()
        
    def setup_mediapipe(self):
        """Initialize MediaPipe hand tracking"""
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=self.MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=self.MIN_TRACKING_CONFIDENCE,
            max_num_hands=2
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
    def find_arduino_port(self):
        """Auto-detect Arduino serial port"""
        ports = list_ports.comports()
        for port in ports:
            # Common Arduino port descriptions
            if any(desc in port.description.lower() for desc in ['arduino', 'ch340', 'cp210x', 'ftdi']):
                return port.device
        return None
        
    def connect_arduino(self):
        """Connect to Arduino for motor control and IR sensor monitoring"""
        arduino_port = self.find_arduino_port()
        if arduino_port is None:
            print("⚠️ Arduino not found. Motor control and IR sensors will be disabled.")
            return False
            
        try:
            self.arduino = serial.Serial(arduino_port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"✅ Arduino connected successfully on port {arduino_port}.")
            self.arduino_connected = True
            return True
        except Exception as e:
            print(f"❌ Could not connect to Arduino on port {arduino_port}: {e}")
            return False
            
    def start_arduino_monitoring(self):
        """Start background thread to monitor Arduino sensors"""
        if not self.arduino_connected:
            return
            
        def monitor_sensors():
            while self.arduino_connected:
                try:
                    if self.arduino.in_waiting > 0:
                        line = self.arduino.readline().decode('utf-8').strip()
                        if line.startswith('IR:'):
                            # Parse IR sensor data: "IR:CLEAR" or "IR:BLOCKED"
                            status = line.split(':')[1]
                            self.ir_sensors_clear = (status == 'CLEAR')
                            if not self.ir_sensors_clear:
                                self.emergency_stop = True
                                print("🚨 EMERGENCY STOP: IR sensors detected obstacle!")
                            else:
                                if self.emergency_stop:
                                    print("✅ IR sensors clear. Operations can resume.")
                                self.emergency_stop = False
                except Exception as e:
                    print(f"Error reading from Arduino: {e}")
                    break
                time.sleep(0.05)  # Check sensors 20 times per second
                
        if self.arduino_connected:
            sensor_thread = threading.Thread(target=monitor_sensors, daemon=True)
            sensor_thread.start()
        
    def find_dobot_port(self):
        """Auto-detect Dobot serial port"""
        ports = list_ports.comports()
        for port in ports:
            if 'Silicon Labs CP210x USB to UART Bridge' in port.description:
                return port.device
        return None
        
    def connect_dobot(self):
        """Connect to the Dobot device"""
        dobot_port = self.find_dobot_port()
        if dobot_port is None:
            print("⚠️ Dobot not found. Please check the connection and try again.")
            return False
            
        try:
            self.dobot = Dobot(port=dobot_port)
            print(f"✅ Dobot connected successfully on port {dobot_port}.")
            # Home the Dobot to ensure it's in a known position
            self.dobot.home()
            print("Dobot homed successfully.")
            # Set initial position
            self.dobot.move_to(*self.HOME_POSITION, wait=True)
            self.dobot_connected = True
            return True
        except Exception as e:
            print(f"❌ Could not open port {dobot_port}: {e}")
            return False
    
    def return_to_home(self):
        """Safely return the Dobot to home position"""
        if not self.dobot_connected or self.returning_home or self.emergency_stop:
            return
            
        try:
            print("Returning to home position...")
            self.returning_home = True
            
            # First move up to avoid collisions
            current_x, current_y, current_z, current_r = self.dobot_x, self.dobot_y, self.dobot_z, self.dobot_r
            safe_z = max(self.DOBOT_Z_RANGE[0] + 50, current_z)
            
            # Move up first
            self.dobot.move_to(current_x, current_y, safe_z, current_r, wait=True)
            time.sleep(0.5)
            
            # Then move to home XY position
            self.dobot.move_to(self.HOME_POSITION[0], self.HOME_POSITION[1], safe_z, current_r, wait=True)
            time.sleep(0.5)
            
            # Finally move to complete home position
            self.dobot.move_to(*self.HOME_POSITION, wait=True)
            
            # Update internal state
            self.dobot_x, self.dobot_y, self.dobot_z, self.dobot_r = self.HOME_POSITION
            self.target_x, self.target_y, self.target_z = self.HOME_POSITION[0], self.HOME_POSITION[1], self.HOME_POSITION[2]
            print("✅ Successfully returned to home position.")
            
            # Reset error flags after successful recovery
            self.in_invalid_zone = False
            self.error_reported = False
            
        except Exception as e:
            print(f"❌ Error returning to home: {e}")
        finally:
            self.returning_home = False
            
    def is_position_invalid(self, x, y, z):
        """Check if a position is in an invalid/inoperable zone"""
        # Check if position is outside allowed ranges
        if not (self.DOBOT_X_RANGE[0] <= x <= self.DOBOT_X_RANGE[1] and
                self.DOBOT_Y_RANGE[0] <= y <= self.DOBOT_Y_RANGE[1] and
                self.DOBOT_Z_RANGE[0] <= z <= self.DOBOT_Z_RANGE[1]):
            return True
            
        # Check if position is in any defined invalid zone
        for min_x, max_x, min_y, max_y, min_z, max_z in self.invalid_zones:
            if (min_x <= x <= max_x and
                min_y <= y <= max_y and
                min_z <= z <= max_z):
                return True
                
        # Additional safety check: positions too close to the base
        if x < 150 and abs(y) < 50 and z < 0:
            return True
            
        return False
        
    def handle_position_error(self, x, y, z, r):
        """Check if position is invalid but allow movement without automatic recovery"""
        try:
            if self.dobot_connected and not self.returning_home and not self.emergency_stop:
                # Check position validity
                if self.is_position_invalid(x, y, z):
                    # Mark as in invalid zone for display purposes
                    self.in_invalid_zone = True
                    
                    # Log the invalid position but don't trigger recovery
                    if not self.error_reported:
                        print(f"⚠️ Invalid position detected: ({x}, {y}, {z})!")
                        self.error_reported = True
                else:
                    # Valid position, reset error flags
                    self.in_invalid_zone = False
                    self.error_reported = False
                    
                # Try to move anyway regardless of validity (but not during emergency stop)
                try:
                    self.dobot.move_to(x, y, z, r, wait=True)
                except Exception as e:
                    print(f"Error when moving to ({x}, {y}, {z}): {e}")
        except Exception as e:
            print(f"Error in handle_position_error: {e}")
            
        return x, y, z, r
        
    def send_motor_command(self, direction):
        """Send motor movement command to Arduino"""
        if not self.arduino_connected or self.emergency_stop:
            return
            
        try:
            if direction != self.motor_direction:
                command = f"MOTOR:{direction}\n"
                self.arduino.write(command.encode())
                self.motor_direction = direction
                print(f"Motor command sent: {direction}")
        except Exception as e:
            print(f"Error sending motor command: {e}")
            
    def process_left_hand_motor_control(self, x, img_width):
        """Process left hand position for motor control"""
        if not self.left_hand_detected:
            # Stop motor when left hand is not detected
            if self.motor_direction != "STOP":
                self.send_motor_command("STOP")
            return
            
        # Calculate left zone boundaries
        left_zone_width = img_width * self.left_zone_width_ratio
        left_boundary = left_zone_width * self.left_zone_boundary_threshold
        right_boundary = left_zone_width * (1 - self.left_zone_boundary_threshold)
        
        # Determine motor direction based on hand position within left zone
        current_time = time.time()
        if current_time - self.last_motor_command_time > self.motor_command_delay:
            if x < left_boundary:
                self.send_motor_command("LEFT")
            elif x > right_boundary:
                self.send_motor_command("RIGHT")
            else:
                self.send_motor_command("STOP")
            self.last_motor_command_time = current_time
            
    def setup_camera(self):
        """Initialize the webcam"""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("❌ Could not open webcam. Please check connection.")
        else:
            # Set higher resolution if supported
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
    def process_hand_landmarks(self, hand_landmarks, img_width, img_height):
        """Extract hand position and gesture information"""
        # Extract the x and y coordinates of the index finger tip (landmark 8)
        x = hand_landmarks.landmark[8].x * img_width
        y = hand_landmarks.landmark[8].y * img_height
        
        # Calculate the distance between the thumb tip and the index finger tip
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        distance = ((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2) ** 0.5
        
        # Detect if thumb and index finger are pinched (for gripper control)
        pinch_threshold = 0.05  # Adjust based on testing
        is_pinched = distance < pinch_threshold
        
        return x, y, distance, is_pinched
        
    def map_hand_to_dobot(self, x, y, hand_label, img_width, img_height):
        """Map hand position to Dobot coordinates"""
        # Skip mapping if in the process of returning home or emergency stop
        if self.returning_home or self.emergency_stop:
            return
            
        if hand_label == "Right":
            # Map vertical position (y) to Dobot's X axis (forward/backward)
            # Invert and scale to match Dobot's coordinate system
            self.target_x = self.DOBOT_X_RANGE[0] + ((1 - (y / img_height)) * 
                           (self.DOBOT_X_RANGE[1] - self.DOBOT_X_RANGE[0]))
            
            # Map horizontal position (x) to Dobot's Y axis (left/right)
            x_center = img_width / 2
            x_relative = x - x_center
            self.target_y = (x_relative / x_center) * self.DOBOT_Y_RANGE[1]
            
        elif hand_label == "Left":
            # Map vertical position (y) to Dobot's Z axis (up/down)
            self.target_z = self.DOBOT_Z_RANGE[0] + ((1 - (y / img_height)) * 
                           (self.DOBOT_Z_RANGE[1] - self.DOBOT_Z_RANGE[0]))
            
    def smooth_motion(self):
        """Apply smoothing to Dobot movement"""
        # Apply exponential smoothing
        self.dobot_x += self.smoothing_factor * (self.target_x - self.dobot_x)
        self.dobot_y += self.smoothing_factor * (self.target_y - self.dobot_y)
        self.dobot_z += self.smoothing_factor * (self.target_z - self.dobot_z)
        
        # Round to integers for Dobot control
        x, y, z, r = int(self.dobot_x), int(self.dobot_y), int(self.dobot_z), self.dobot_r
        
        return x, y, z, r
        
    def toggle_air_pump(self, state):
        """Turn the air pump on or off"""
        if not self.dobot_connected or self.returning_home or self.emergency_stop:
            return
            
        try:
            if state:
                self.dobot.suck(True)
                print("Air pump turned ON")
            else:
                self.dobot.suck(False)
                print("Air pump turned OFF")
            self.air_pump_on = state
        except Exception as e:
            print(f"Error controlling air pump: {e}")
            
    def draw_left_hand_control_zone(self, frame):
        """Draw the left hand control zone when left hand is detected"""
        if not self.left_hand_detected:
            return
            
        img_height, img_width = frame.shape[:2]
        left_zone_width = int(img_width * self.left_zone_width_ratio)
        
        # Draw the control zone box
        zone_color = (0, 255, 255) if not self.emergency_stop else (0, 0, 255)  # Yellow or Red
        cv2.rectangle(frame, (0, 0), (left_zone_width, img_height), zone_color, 3)
        
        # Draw boundary lines
        left_boundary = int(left_zone_width * self.left_zone_boundary_threshold)
        right_boundary = int(left_zone_width * (1 - self.left_zone_boundary_threshold))
        
        cv2.line(frame, (left_boundary, 0), (left_boundary, img_height), (255, 0, 0), 2)
        cv2.line(frame, (right_boundary, 0), (right_boundary, img_height), (255, 0, 0), 2)
        
        # Add labels
        cv2.putText(frame, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "MOTOR", (left_zone_width//2 - 30, img_height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "RIGHT", (right_boundary + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Show current motor direction
        direction_color = (0, 255, 0) if self.motor_direction == "STOP" else (0, 165, 255)
        cv2.putText(frame, f"Motor: {self.motor_direction}", 
                   (10, img_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)
            
    def draw_info_overlay(self, frame, hand_data=None):
        """Draw information overlay on the video frame"""
        # Emergency stop indicator
        if self.emergency_stop:
            cv2.putText(frame, "🚨 EMERGENCY STOP - IR SENSOR TRIGGERED 🚨", 
                       (frame.shape[1]//2 - 300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        
        # Draw Dobot position
        position_color = (0, 0, 255) if self.in_invalid_zone or self.emergency_stop else (0, 255, 0)
        cv2.putText(frame, f"Dobot: X={int(self.dobot_x)}, Y={int(self.dobot_y)}, Z={int(self.dobot_z)}", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, position_color, 2)
        
        # Draw connection status
        dobot_status = "CONNECTED" if self.dobot_connected else "DISCONNECTED"
        arduino_status = "CONNECTED" if self.arduino_connected else "DISCONNECTED"
        dobot_color = (0, 255, 0) if self.dobot_connected else (0, 0, 255)
        arduino_color = (0, 255, 0) if self.arduino_connected else (0, 0, 255)
        
        cv2.putText(frame, f"Dobot: {dobot_status}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dobot_color, 2)
        cv2.putText(frame, f"Arduino: {arduino_status}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, arduino_color, 2)
        
        # Draw position status
        position_status = "INVALID POSITION" if self.in_invalid_zone else "VALID POSITION"
        position_color = (0, 0, 255) if self.in_invalid_zone else (0, 255, 0)
        cv2.putText(frame, f"Position: {position_status}", 
                   (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, position_color, 2)
        
        # Draw air pump status
        pump_status = "ON" if self.air_pump_on else "OFF"
        pump_color = (0, 0, 255) if self.air_pump_on else (255, 0, 0)
        cv2.putText(frame, f"Air Pump: {pump_status}", 
                   (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, pump_color, 2)
        
        # Draw IR sensor status
        ir_status = "CLEAR" if self.ir_sensors_clear else "BLOCKED"
        ir_color = (0, 255, 0) if self.ir_sensors_clear else (0, 0, 255)
        cv2.putText(frame, f"IR Sensors: {ir_status}", 
                   (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ir_color, 2)
        
        # Draw returning home status if active
        if self.returning_home:
            cv2.putText(frame, "RETURNING TO HOME POSITION", 
                       (frame.shape[1]//2 - 200, frame.shape[0]//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # Draw hand data if available
        if hand_data:
            x, y, distance, is_pinched = hand_data
            cv2.putText(frame, f"Hand: X={int(x)}, Y={int(y)}", 
                       (10, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(frame, f"Pinch: {'YES' if is_pinched else 'NO'}", 
                       (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        
        # Add control instructions
        cv2.putText(frame, "Controls: Right hand = X/Y, Left hand = Z + Motor Base, Pinch = Toggle pump, 'q' = Quit, 'h' = Home", 
                   (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Motor Control: Left hand in yellow zone - move to edges to control base movement", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                   
        # Draw invalid zones visualization in 2D (top-down view)
        self.draw_invalid_zones(frame)
                   
    def draw_invalid_zones(self, frame):
        """Draw a visualization of invalid zones on the frame"""
        # Create a small 2D map in the bottom right corner
        map_width, map_height = 200, 200
        map_x, map_y = frame.shape[1] - map_width - 10, frame.shape[0] - map_height - 50
        
        # Draw map background
        cv2.rectangle(frame, (map_x, map_y), (map_x + map_width, map_y + map_height), (50, 50, 50), -1)
        cv2.rectangle(frame, (map_x, map_y), (map_x + map_width, map_y + map_height), (200, 200, 200), 2)
        
        # Draw coordinate system
        cv2.line(frame, (map_x + map_width//2, map_y), (map_x + map_width//2, map_y + map_height), (100, 100, 100), 1)
        cv2.line(frame, (map_x, map_y + map_height//2), (map_x + map_width, map_y + map_height//2), (100, 100, 100), 1)
        
        # Draw dobot base
        center_x, center_y = map_x + map_width//2, map_y + map_height//2
        cv2.circle(frame, (center_x, center_y), 10, (0, 128, 255), -1)
        
        # Scale factors for mapping real coordinates to map
        x_scale = map_width / (self.DOBOT_X_RANGE[1] - self.DOBOT_X_RANGE[0])
        y_scale = map_height / (self.DOBOT_Y_RANGE[1] - self.DOBOT_Y_RANGE[0])
        
        # Draw invalid zones (top view, X and Y only)
        for min_x, max_x, min_y, max_y, _, _ in self.invalid_zones:
            # Map coordinates to map space
            rect_x1 = center_x + int((min_y - 0) * y_scale)  # Y axis is horizontal in map
            rect_y1 = center_y - int((min_x - self.DOBOT_X_RANGE[0]) * x_scale)  # X axis is vertical in map
            rect_x2 = center_x + int((max_y - 0) * y_scale)
            rect_y2 = center_y - int((max_x - self.DOBOT_X_RANGE[0]) * x_scale)
            
            # Draw invalid zone
            cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 255), -1)
            cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (255, 255, 255), 1)
        
        # Draw current dobot position
        pos_x = center_x + int((self.dobot_y - 0) * y_scale)
        pos_y = center_y - int((self.dobot_x - self.DOBOT_X_RANGE[0]) * x_scale)
        pos_color = (0, 0, 255) if self.in_invalid_zone else (0, 255, 0)
        cv2.circle(frame, (pos_x, pos_y), 5, pos_color, -1)
        
        # Draw label
        cv2.putText(frame, "Dobot Map (Top View)", (map_x, map_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                   
    def run(self):
        """Main execution loop"""
        if not self.cap.isOpened():
            print("Camera not available. Exiting.")
            return
            
        try:
            while self.cap.isOpened():
                success, frame = self.cap.read()
                if not success:
                    print("Failed to capture video. Exiting.")
                    break
                    
                # Mirror the frame horizontally for more intuitive control
                frame = cv2.flip(frame, 1)
                
                # Convert to RGB for MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Process the frame for hand detection
                results = self.hands.process(rgb_frame)
                
                # Reset left hand detection flag
                self.left_hand_detected = False
                
                hand_data = None
                if results.multi_hand_landmarks and not self.returning_home:
                    for idx, (hand_landmarks, hand_handedness) in enumerate(
                            zip(results.multi_hand_landmarks, results.multi_handedness)):
                        
                        hand_label = hand_handedness.classification[0].label
                        img_height, img_width, _ = frame.shape
                        
                        # Process landmarks to get position and gesture
                        x, y, distance, is_pinched = self.process_hand_landmarks(
                            hand_landmarks, img_width, img_height)
                        
                        # Store hand data for overlay display
                        hand_data = (x, y, distance, is_pinched)
                        
                        if hand_label == "Left":
                            self.left_hand_detected = True
                            # Check if left hand is in the control zone for motor control
                            left_zone_width = img_width * self.left_zone_width_ratio
                            if x <= left_zone_width:
                                self.process_left_hand_motor_control(x, img_width)
                        
                        # Map hand position to Dobot coordinates (existing functionality)
                        self.map_hand_to_dobot(x, y, hand_label, img_width, img_height)
                        
                        # Toggle air pump based on pinch gesture (existing functionality)
                        if is_pinched and time.time() - self.last_command_time > 1.0:
                            self.toggle_air_pump(not self.air_pump_on)
                            self.last_command_time = time.time()
                        
                        # Draw hand landmarks on the frame
                        self.mp_drawing.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # If no left hand detected, stop motor
                if not self.left_hand_detected and self.motor_direction != "STOP":
                    self.send_motor_command("STOP")
                
                # Apply movement smoothing and get target position
                x, y, z, r = self.smooth_motion()
                
                # Send movement command to Dobot if enough time has passed and not in emergency stop
                if (self.dobot_connected and not self.emergency_stop and 
                    time.time() - self.last_command_time > self.COMMAND_DELAY):
                    # Check if position is invalid but still try to move there
                    x, y, z, r = self.handle_position_error(x, y, z, r)
                    self.last_command_time = time.time()
                
                # Draw left hand control zone if left hand is detected
                self.draw_left_hand_control_zone(frame)
                
                # Draw information overlay
                self.draw_info_overlay(frame, hand_data)
                
                # Show the frame
                cv2.imshow('Dobot Hand Controller with Arduino Base Control', frame)
                
                # Check for key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):  # Press 'h' to manually return to home
                    self.return_to_home()
                elif key == ord('s'):  # Press 's' to stop motor manually
                    self.send_motor_command("STOP")
                elif key == ord('r'):  # Press 'r' to reset emergency stop (if sensors are clear)
                    if self.ir_sensors_clear:
                        self.emergency_stop = False
                        print("Emergency stop reset.")
                    
        except KeyboardInterrupt:
            print("Operation interrupted by user")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Release resources and perform clean shutdown"""
        print("Shutting down...")
        
        # Stop motor
        if self.arduino_connected:
            self.send_motor_command("STOP")
        
        # Turn off air pump if it's on
        if self.dobot_connected and self.air_pump_on:
            self.toggle_air_pump(False)
        
        # Return Dobot to home position before disconnecting
        if self.dobot_connected and not self.emergency_stop:
            try:
                self.dobot.move_to(*self.HOME_POSITION, wait=True)
                print("Dobot returned to home position")
            except Exception as e:
                print(f"Error returning Dobot to home position: {e}")
            
            # Close Dobot connection
            self.dobot.close()
        
        # Close Arduino connection
        if self.arduino_connected:
            try:
                self.arduino.close()
                print("Arduino connection closed")
            except Exception as e:
                print(f"Error closing Arduino connection: {e}")
        
        # Release camera
        if self.cap.isOpened():
            self.cap.release()
            
        # Close all OpenCV windows
        cv2.destroyAllWindows()
        print("Cleanup complete")

# Create and run the controller
if __name__ == "__main__":
    controller = DobotHandController()
    controller.run()
