import cv2
import mediapipe as mp
import time
from serial.tools import list_ports
import serial
import threading
import keyboard

# Try to import Dobot, but handle if not available
try:
    from pydobotplus import Dobot
    DOBOT_AVAILABLE = True
except ImportError:
    print("pydobotplus not installed - Dobot functionality disabled")
    DOBOT_AVAILABLE = False

class DobotHandController:
    def __init__(self):
        # Configuration
        key1 = cv2.waitKey(1) & 0xFF

        self.COMMAND_DELAY = 0.2

        # Keyboard control variables (add after line ~48)
        self.keyboard_running = True
        self.last_key_states = {'j': False, 'l': False, 'k': False}
        
        # State variables
        self.dobot_x, self.dobot_y, self.dobot_z, self.dobot_r = (200, 0, 0, 0)
        self.last_command_time = time.time()
        self.air_pump_on = False
        self.dobot = None
        self.dobot_connected = False

        # Air pump delay variables
        self.pump_gesture_last_seen_time = 0
        self.pump_delay_duration = 3.0  # 3 seconds delay
        
        # Arduino variables
        self.arduino = None
        self.arduino_connected = False
        self.motor_direction = "STOP"
        self.left_hand_detected = False
        self.right_hand_detected = False
        self.hand_in_left_zone = False
        self.hand_in_right_zone = False
        
        # Gesture control variables
        self.left_hand_gesture_active = False
        self.right_hand_gesture_active = False
        self.left_hand_pump_gesture_active = False
        self.right_hand_pump_gesture_active = False
        
        # Control zone parameters - NEW LAYOUT
        self.left_zone_width_ratio = 0.3   # 30% left zone
        self.center_zone_width_ratio = 0.4 # 40% center zone
        self.right_zone_width_ratio = 0.3  # 30% right zone
        self.zone_trigger_margin = 0.1     # 10% margin from edges
        
        # Initialize components
        self.setup_mediapipe()
        self.connect_dobot()  # Connect Dobot first
        self.connect_arduino()  # Then Arduino
        self.setup_camera()
        
        if self.arduino_connected:
            self.start_arduino_monitoring()
        if self.arduino_connected:
           self.start_keyboard_monitoring()
        
    def setup_mediapipe(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=2
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
    def check_thumb_pinky_gesture(self, hand_landmarks):
        """
        Check if thumb and pinky are extended while other fingers are folded
        Returns True if the gesture is detected
        """
        # Landmark indices for fingertips and MCPs (base joints)
        THUMB_TIP = 4
        THUMB_MCP = 2
        INDEX_TIP = 8
        INDEX_PIP = 6
        MIDDLE_TIP = 12
        MIDDLE_PIP = 10
        RING_TIP = 16
        RING_PIP = 14
        PINKY_TIP = 20
        PINKY_MCP = 17
        
        # Get landmark positions
        landmarks = hand_landmarks.landmark
        
        # Check if thumb is extended (tip higher than MCP joint)
        thumb_extended = landmarks[THUMB_TIP].y < landmarks[THUMB_MCP].y
        
        # Check if pinky is extended (tip higher than MCP joint)
        pinky_extended = landmarks[PINKY_TIP].y < landmarks[PINKY_MCP].y
        
        # Check if index finger is folded (tip lower than PIP joint)
        index_folded = landmarks[INDEX_TIP].y > landmarks[INDEX_PIP].y
        
        # Check if middle finger is folded (tip lower than PIP joint)
        middle_folded = landmarks[MIDDLE_TIP].y > landmarks[MIDDLE_PIP].y
        
        # Check if ring finger is folded (tip lower than PIP joint)
        ring_folded = landmarks[RING_TIP].y > landmarks[RING_PIP].y
        
        # Return True only if thumb and pinky are extended AND other fingers are folded
        return (thumb_extended and pinky_extended and 
                index_folded and middle_folded and ring_folded)

    def check_thumb_index_middle_gesture(self, hand_landmarks):
        """
        Check if thumb, index, and middle fingers are extended while ring and pinky are folded
        Returns True if the gesture is detected
        """
        # Landmark indices for fingertips and MCPs/PIPs (base/middle joints)
        THUMB_TIP = 4
        THUMB_MCP = 2
        INDEX_TIP = 8
        INDEX_PIP = 6
        MIDDLE_TIP = 12
        MIDDLE_PIP = 10
        RING_TIP = 16
        RING_PIP = 14
        PINKY_TIP = 20
        PINKY_MCP = 17
        
        # Get landmark positions
        landmarks = hand_landmarks.landmark
        
        # Check if thumb is extended (tip higher than MCP joint)
        thumb_extended = landmarks[THUMB_TIP].y < landmarks[THUMB_MCP].y
        
        # Check if index finger is extended (tip higher than PIP joint)
        index_extended = landmarks[INDEX_TIP].y < landmarks[INDEX_PIP].y
        
        # Check if middle finger is extended (tip higher than PIP joint)
        middle_extended = landmarks[MIDDLE_TIP].y < landmarks[MIDDLE_PIP].y
        
        # Check if ring finger is folded (tip lower than PIP joint)
        ring_folded = landmarks[RING_TIP].y > landmarks[RING_PIP].y
        
        # Check if pinky is folded (tip lower than MCP joint)
        pinky_folded = landmarks[PINKY_TIP].y > landmarks[PINKY_MCP].y
        
        # Return True only if thumb, index, and middle are extended AND ring and pinky are folded
        return (thumb_extended and index_extended and middle_extended and 
                ring_folded and pinky_folded)
        
    def find_arduino_port(self):
        try:
            # Get list of all ports
            ports = list_ports.comports()
            
            # First, identify the Dobot port to exclude it
            dobot_port = None
            for port in ports:
                if 'Silicon Labs CP210x USB to UART Bridge' in port.description:
                    dobot_port = port.device
                    break
            
            # Now find Arduino, excluding the Dobot port
            for port in ports:
                # Skip the Dobot port
                if port.device == dobot_port:
                    continue
                    
                # Look for Arduino-specific identifiers
                description_lower = port.description.lower()
                
                # More specific Arduino detection
                arduino_indicators = [
                    'arduino',
                    'ch340',  # Common Arduino clone chip
                    'ftdi',   # FTDI chips used in some Arduinos
                ]
                
                # Additional check for CP210x that's NOT the Dobot
                if 'cp210x' in description_lower and 'silicon labs' not in port.description:
                    return port.device
                    
                # Check for Arduino indicators
                if any(indicator in description_lower for indicator in arduino_indicators):
                    return port.device
                    
                # Check for common Arduino vendor IDs
                if hasattr(port, 'vid') and port.vid:
                    # Common Arduino vendor IDs
                    arduino_vids = [0x2341, 0x1A86, 0x0403]  # Arduino, CH340, FTDI
                    if port.vid in arduino_vids:
                        return port.device
                        
        except Exception as e:
            print(f"Error finding Arduino port: {e}")
        return None
        
    def connect_arduino(self):
        try:
            arduino_port = self.find_arduino_port()
            if arduino_port is None:
                print("Arduino not found - motor control disabled")
                return False
                
            self.arduino = serial.Serial(arduino_port, 9600, timeout=1)
            time.sleep(2)
            print(f"Arduino connected on {arduino_port}")
            self.arduino_connected = True
            return True
        except Exception as e:
            print(f"Arduino connection failed: {e}")
            self.arduino_connected = False
            return False
            
    def start_arduino_monitoring(self):
        def monitor_sensors():
            while self.arduino_connected:
                try:
                    if self.arduino and self.arduino.in_waiting > 0:
                        line = self.arduino.readline().decode('utf-8').strip()
                        if line.startswith('IR:'):
                            status = line.split(':')[1]
                            if status == 'BLOCKED':
                                print("IR sensor blocked - stopping motor")
                except Exception as e:
                    print(f"Arduino monitoring error: {e}")
                    break
                time.sleep(0.05)
                
        sensor_thread = threading.Thread(target=monitor_sensors, daemon=True)
        sensor_thread.start()
    def start_keyboard_monitoring(self):
       keyboard_thread = threading.Thread(target=self.keyboard_monitor_thread, daemon=True)
       keyboard_thread.start()
       print("Keyboard control started: 'j'=Left, 'l'=Right, 'k'=Stop")
    
        
    def find_dobot_port(self):
        try:
            ports = list_ports.comports()
            for port in ports:
                if 'Silicon Labs CP210x USB to UART Bridge' in port.description:
                    return port.device
        except Exception as e:
            print(f"Error finding Dobot port: {e}")
        return None
        
    def connect_dobot(self):
        if not DOBOT_AVAILABLE:
            print("Dobot library not available - arm control disabled")
            return False
            
        try:
            dobot_port = self.find_dobot_port()
            if dobot_port is None:
                print("Dobot not found - arm control disabled")
                return False
                
            self.dobot = Dobot(port=dobot_port)
            print(f"Dobot connected on {dobot_port}")
            # Removed homing - Dobot will start from current position
            self.dobot_connected = True
            return True
        except Exception as e:
            print(f"Dobot connection failed: {e}")
            self.dobot_connected = False
            self.dobot = None
            return False
            
    def send_motor_command(self, direction):
     if not self.arduino_connected or not self.arduino:
        print(f"Stepper command '{direction}' - Arduino not connected")
        return
    
     try:
        if direction != self.motor_direction:
            command = f"MOTOR:{direction}\n"
            self.arduino.write(command.encode())
            self.motor_direction = direction
            print(f"Stepper Motor: {direction}")
            
            # Add LED status info
            if direction == "j":
                print("LED: Blinking during rotation")
            elif direction == "l":
                print("LED: Constant ON")
            elif direction == "k":
                print("LED: OFF")
                
     except Exception as e:
        print(f"Stepper command error: {e}")
        self.arduino_connected = False

    def keyboard_monitor_thread(self):
     while self.keyboard_running:
        try:
            j_pressed = keyboard.is_pressed('j')
            l_pressed = keyboard.is_pressed('l')
            k_pressed = keyboard.is_pressed('k')
            
            # Check for key press events (not held down)
            j_just_pressed = j_pressed and not self.last_key_states['j']
            l_just_pressed = l_pressed and not self.last_key_states['l']
            k_just_pressed = k_pressed and not self.last_key_states['k']
            
            # Start forward motion on 'j' press
            if j_just_pressed:
                self.send_motor_command("j")
                print("Keyboard: Forward - STARTED")
            
            # Start backward motion on 'l' press
            elif l_just_pressed:
                self.send_motor_command("l")
                print("Keyboard: Backward - STARTED")
            
            # Stop motor on 'k' press
            elif k_just_pressed:
                self.send_motor_command("k")
                print("Keyboard: Motor - STOPPED")
            
            # Update last key states
            self.last_key_states['j'] = j_pressed
            self.last_key_states['l'] = l_pressed
            self.last_key_states['k'] = k_pressed
            
            time.sleep(0.02)
            
        except Exception as e:
            print(f"Error in keyboard monitor: {e}")
            break
            
    def process_left_hand_motor_control(self, x, img_width):
        # Only control motor if either gesture is active
        if not self.left_hand_detected or (not self.left_hand_gesture_active and not self.left_hand_pump_gesture_active):
            if self.motor_direction != "k":
                self.send_motor_command("k")
            return
            
        # NEW ZONE LAYOUT: Left 30% | Center 40% | Right 30%
        left_zone_width = img_width * self.left_zone_width_ratio
        center_zone_start = left_zone_width
        center_zone_end = left_zone_width + (img_width * self.center_zone_width_ratio)
        
        # Trigger boundaries within left and right zones
        left_trigger = left_zone_width * self.zone_trigger_margin
        left_stop_trigger = left_zone_width * (1 - self.zone_trigger_margin)
        right_start_trigger = center_zone_end + ((img_width - center_zone_end) * self.zone_trigger_margin)
        right_trigger = center_zone_end + ((img_width - center_zone_end) * (1 - self.zone_trigger_margin))
        
        # Left zone (0% to 30%)
        if x < left_zone_width:
            if x < left_trigger:
                if not self.hand_in_left_zone:
                    self.send_motor_command("LEFT")
                    self.hand_in_left_zone = True
                    self.hand_in_right_zone = False
            elif x > left_stop_trigger:
                if self.hand_in_left_zone:
                    self.send_motor_command("STOP")
                    self.hand_in_left_zone = False
        # Center zone (30% to 70%) - Z-axis control only, no motor
        elif x < center_zone_end:
            if self.hand_in_left_zone or self.hand_in_right_zone:
                self.send_motor_command("STOP")
                self.hand_in_left_zone = False
                self.hand_in_right_zone = False
        # Right zone (70% to 100%)
        else:
            if x < right_start_trigger:
                if self.hand_in_right_zone:
                    self.send_motor_command("STOP")
                    self.hand_in_right_zone = False
            elif x > right_trigger:
                if not self.hand_in_right_zone:
                    self.send_motor_command("RIGHT")
                    self.hand_in_right_zone = True
                    self.hand_in_left_zone = False
            
    def setup_camera(self):
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Camera not available - trying alternative indices...")
                # Try other camera indices
                for i in range(1, 5):
                    self.cap = cv2.VideoCapture(i)
                    if self.cap.isOpened():
                        print(f"Camera found at index {i}")
                        break
                else:
                    print("No camera found")
                    return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            return True
        except Exception as e:
            print(f"Camera setup error: {e}")
            return False
            
    def process_hand_landmarks(self, hand_landmarks, img_width, img_height):
        # Get index finger tip position
        x = hand_landmarks.landmark[8].x * img_width
        y = hand_landmarks.landmark[8].y * img_height
        
        # Check for pinch gesture (for manual air pump control)
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        distance = ((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2) ** 0.5
        is_pinched = distance < 0.05
        
        # Check for both gestures
        thumb_pinky_gesture = self.check_thumb_pinky_gesture(hand_landmarks)
        thumb_index_middle_gesture = self.check_thumb_index_middle_gesture(hand_landmarks)
        
        return x, y, is_pinched, thumb_pinky_gesture, thumb_index_middle_gesture
        
    def map_hand_to_dobot(self, x, y, hand_label, img_width, img_height):
        # Only update coordinates if any appropriate gesture is active
        if hand_label == "Right" and (self.right_hand_gesture_active or self.right_hand_pump_gesture_active):
            # Map right hand to X and Y axes
            self.dobot_x = 100 + ((1 - (y / img_height)) * 200)  # X: 100-300
            x_center = img_width / 2
            x_relative = x - x_center
            self.dobot_y = (x_relative / x_center) * 200  # Y: -200 to 200
            
        elif hand_label == "Left" and (self.left_hand_gesture_active or self.left_hand_pump_gesture_active):
            # Map left hand to Z axis (when in center zone)
            left_zone_width = img_width * self.left_zone_width_ratio
            center_zone_end = left_zone_width + (img_width * self.center_zone_width_ratio)
            
            # Only control Z when in center zone (30% to 70%)
            if left_zone_width <= x <= center_zone_end:
                self.dobot_z = -78 + ((1 - (y / img_height)) * 218)  # Z: -78 to 140
                
    def toggle_air_pump(self, state):
        if not self.dobot_connected or not self.dobot:
            print(f"Air pump {'ON' if state else 'OFF'} - Dobot not connected")
            self.air_pump_on = state  # Update virtual state
            return
            
        try:
            self.dobot.suck(state)
            self.air_pump_on = state
            print(f"Air pump: {'ON' if state else 'OFF'}")
        except Exception as e:
            print(f"Air pump error: {e}")
            self.dobot_connected = False  # Mark as disconnected on error

    def manage_air_pump_state(self):

    # Check if any pump gesture is currently active
       pump_gesture_active = self.left_hand_pump_gesture_active or self.right_hand_pump_gesture_active
    
    # Update last seen time if gesture is active
       if pump_gesture_active:
        self.pump_gesture_last_seen_time = time.time()
        # Turn on pump immediately if gesture is active and pump is off
        if not self.air_pump_on:
            self.toggle_air_pump(True)
       else:
        # If no pump gesture is active, check if delay period has passed
        time_since_last_gesture = time.time() - self.pump_gesture_last_seen_time
        if self.air_pump_on and time_since_last_gesture >= self.pump_delay_duration:
            self.toggle_air_pump(False)
            
    def draw_interface(self, frame):
        # Status info at top left
        y_pos = 30
        
        # Always show coordinates (virtual or real)
        status_color = (0, 255, 0) if self.dobot_connected else (100, 100, 100)
        cv2.putText(frame, f"Dobot: {int(self.dobot_x)}, {int(self.dobot_y)}, {int(self.dobot_z)}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        y_pos += 30
        
        pump_color = (0, 255, 0) if self.dobot_connected else (100, 100, 100)
        cv2.putText(frame, f"Pump: {'ON' if self.air_pump_on else 'OFF'}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pump_color, 2)
        y_pos += 30
        
        # Motor status
        led_status = "OFF"
        if self.motor_direction == "LEFT":
            led_status = "BLINKING"
        elif self.motor_direction == "RIGHT":
            led_status = "ON"
            
        motor_color = (0, 255, 0) if self.arduino_connected else (100, 100, 100)
        cv2.putText(frame, f"Stepper: {self.motor_direction} | LED: {led_status}", 
           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, motor_color, 2)
        y_pos += 30
        
        # Gesture status
        left_gesture_color = (0, 255, 0) if self.left_hand_gesture_active else (255, 255, 255)
        left_pump_gesture_color = (255, 0, 0) if self.left_hand_pump_gesture_active else (255, 255, 255)
        right_gesture_color = (0, 255, 0) if self.right_hand_gesture_active else (255, 255, 255)
        right_pump_gesture_color = (255, 0, 0) if self.right_hand_pump_gesture_active else (255, 255, 255)
        
        cv2.putText(frame, f"Left Normal: {'ACTIVE' if self.left_hand_gesture_active else 'INACTIVE'}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, left_gesture_color, 2)
        y_pos += 25
        cv2.putText(frame, f"Left Pump: {'ACTIVE' if self.left_hand_pump_gesture_active else 'INACTIVE'}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, left_pump_gesture_color, 2)
        y_pos += 25
        cv2.putText(frame, f"Right Normal: {'ACTIVE' if self.right_hand_gesture_active else 'INACTIVE'}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, right_gesture_color, 2)
        y_pos += 25
        cv2.putText(frame, f"Right Pump: {'ACTIVE' if self.right_hand_pump_gesture_active else 'INACTIVE'}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, right_pump_gesture_color, 2)
        y_pos += 25
        
        # Connection status
        dobot_status = "Connected" if self.dobot_connected else "Disconnected"
        arduino_status = "Connected" if self.arduino_connected else "Disconnected"
        cv2.putText(frame, f"Dobot: {dobot_status} | Arduino: {arduino_status}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_pos += 30
        
        # Draw NEW control zones
        if self.left_hand_detected:
            img_height, img_width = frame.shape[:2]
            
            # Calculate zone boundaries
            left_zone_width = int(img_width * self.left_zone_width_ratio)
            center_zone_start = left_zone_width
            center_zone_end = int(left_zone_width + (img_width * self.center_zone_width_ratio))
            
            # Left zone (30% - Motor Left)
            left_zone_color = (0, 255, 255) if self.arduino_connected else (100, 100, 100)
            if self.left_hand_gesture_active or self.left_hand_pump_gesture_active:
                left_zone_color = (0, 255, 0) if self.left_hand_gesture_active else (0, 0, 255)
            cv2.rectangle(frame, (0, 0), (left_zone_width, img_height), left_zone_color, 2)
            cv2.putText(frame, "LEFT", (10, img_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Center zone (40% - Z-axis control)
            center_zone_color = (255, 255, 0) if self.arduino_connected else (100, 100, 100)
            if self.left_hand_gesture_active or self.left_hand_pump_gesture_active:
                center_zone_color = (0, 255, 0) if self.left_hand_gesture_active else (0, 0, 255)
            cv2.rectangle(frame, (center_zone_start, 0), (center_zone_end, img_height), center_zone_color, 2)
            cv2.putText(frame, "Z-AXIS", (center_zone_start + 20, img_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Right zone (30% - Motor Right)  
            right_zone_color = (0, 255, 255) if self.arduino_connected else (100, 100, 100)
            if self.left_hand_gesture_active or self.left_hand_pump_gesture_active:
                right_zone_color = (0, 255, 0) if self.left_hand_gesture_active else (0, 0, 255)
            cv2.rectangle(frame, (center_zone_end, 0), (img_width, img_height), right_zone_color, 2)
            cv2.putText(frame, "RIGHT", (center_zone_end + 20, img_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Draw trigger lines
            left_trigger = int(left_zone_width * self.zone_trigger_margin)
            left_stop_trigger = int(left_zone_width * (1 - self.zone_trigger_margin))
            right_start_trigger = int(center_zone_end + ((img_width - center_zone_end) * self.zone_trigger_margin))
            right_trigger = int(center_zone_end + ((img_width - center_zone_end) * (1 - self.zone_trigger_margin)))
            
            cv2.line(frame, (left_trigger, 0), (left_trigger, img_height), (255, 0, 0), 2)
            cv2.line(frame, (left_stop_trigger, 0), (left_stop_trigger, img_height), (255, 0, 0), 2)
            cv2.line(frame, (right_start_trigger, 0), (right_start_trigger, img_height), (255, 0, 0), 2)
            cv2.line(frame, (right_trigger, 0), (right_trigger, img_height), (255, 0, 0), 2)
        
        # Controls at bottom
        cv2.putText(frame, "Left: 30% | Center: 40% | Right: 30% | Thumb+Pinky: Normal | Thumb+Index+Middle: With Pump | 'q': Quit", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                   
    def run(self):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            print("Camera not available - cannot start")
            return
            
        print("Starting controller...")
        print(f"Dobot: {'Connected' if self.dobot_connected else 'Not connected'}")
        print(f"Arduino: {'Connected' if self.arduino_connected else 'Not connected'}")
        print("Camera: Connected")
        print("NEW Zone Layout:")
        print("  - Left 30%: Motor LEFT control")
        print("  - Center 40%: Z-axis control")
        print("  - Right 30%: Motor RIGHT control")
        print("Gestures:")
        print("  - Thumb + Pinky: Enable robot movement (no air pump)")
        print("  - Thumb + Index + Middle: Enable robot movement with air pump")
        print("  - Pinch: Manual toggle air pump")
        
        if not self.dobot_connected and not self.arduino_connected:
            print("Running in demo mode - hand tracking only")
        
        try:
            while self.cap.isOpened():
                success, frame = self.cap.read()
                if not success:
                    break
                    
                frame = cv2.flip(frame, 1)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_frame)
                
                # Reset hand detection flags
                self.left_hand_detected = False
                self.right_hand_detected = False
                self.left_hand_gesture_active = False
                self.right_hand_gesture_active = False
                self.left_hand_pump_gesture_active = False
                self.right_hand_pump_gesture_active = False
                
                if results.multi_hand_landmarks:
                    for hand_landmarks, hand_handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                        hand_label = hand_handedness.classification[0].label
                        img_height, img_width, _ = frame.shape
                        
                        x, y, is_pinched, thumb_pinky_gesture, thumb_index_middle_gesture = self.process_hand_landmarks(
                            hand_landmarks, img_width, img_height)
                        
                        if hand_label == "Left":
                            self.left_hand_detected = True
                            self.left_hand_gesture_active = thumb_pinky_gesture
                            self.left_hand_pump_gesture_active = thumb_index_middle_gesture
                            self.process_left_hand_motor_control(x, img_width)
                        elif hand_label == "Right":
                            self.right_hand_detected = True
                            self.right_hand_gesture_active = thumb_pinky_gesture
                            self.right_hand_pump_gesture_active = thumb_index_middle_gesture
                        
                        # Only update coordinates if any gesture is active
                        self.map_hand_to_dobot(x, y, hand_label, img_width, img_height)
                        
                        # Manual toggle air pump on pinch (only if any gesture is active)
                        if (is_pinched and time.time() - self.last_command_time > 1.0 and 
                            ((hand_label == "Left" and (self.left_hand_gesture_active or self.left_hand_pump_gesture_active)) or
                             (hand_label == "Right" and (self.right_hand_gesture_active or self.right_hand_pump_gesture_active)))):
                            self.toggle_air_pump(not self.air_pump_on)
                            self.last_command_time = time.time()
                        
                        # Draw hand landmarks with color based on gesture
                        if thumb_index_middle_gesture:
                            connection_color = (0, 0, 255)  # Red for pump gesture
                        elif thumb_pinky_gesture:
                            connection_color = (0, 255, 0)  # Green for normal gesture
                        else:
                            connection_color = (255, 255, 255)  # White for no gesture
                            
                        self.mp_drawing.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                            landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=connection_color, thickness=2, circle_radius=2),
                            connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=connection_color, thickness=2)
                        )
                
                # Manage air pump state based on gestures
                self.manage_air_pump_state()
                
                # Stop motor if no left hand or no gesture active
                if (not self.left_hand_detected or 
                    (not self.left_hand_gesture_active and not self.left_hand_pump_gesture_active)) and self.motor_direction != "STOP":
                    self.send_motor_command("STOP")
                    self.hand_in_left_zone = False
                    self.hand_in_right_zone = False
                
                # Move Dobot (only if connected and any gesture is active)
                if (self.dobot_connected and self.dobot and 
                    (self.left_hand_gesture_active or self.right_hand_gesture_active or 
                     self.left_hand_pump_gesture_active or self.right_hand_pump_gesture_active) and
                    time.time() - self.last_command_time > self.COMMAND_DELAY):
                    try:
                        self.dobot.move_to(int(self.dobot_x), int(self.dobot_y), int(self.dobot_z), self.dobot_r, wait=False)
                    except Exception as e:
                        print(f"Dobot move error: {e}")
                        self.dobot_connected = False  # Mark as disconnected on error
                    self.last_command_time = time.time()
                
                self.draw_interface(frame)
                cv2.imshow('Dobot Hand Controller', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        print("Shutting down...")
        self.keyboard_running = False
        
        if self.arduino_connected and self.arduino:
            try:
                self.send_motor_command("STOP")
                self.arduino.close()
            except Exception as e:
                print(f"Arduino cleanup error: {e}")
        
        if self.dobot_connected and self.dobot:
            try:
                if self.air_pump_on:
                    self.toggle_air_pump(False)
                # Removed homing on cleanup - just close connection
                self.dobot.close()
            except Exception as e:
                print(f"Dobot cleanup error: {e}")
        
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    controller = DobotHandController()
    controller.run()
