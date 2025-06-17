import cv2
import mediapipe as mp
import time
from pydobotplus import Dobot
from serial.tools import list_ports
import serial
import threading

class DobotHandController:
    def __init__(self):
        # Configuration
        self.COMMAND_DELAY = 0.5
        self.HOME_POSITION = (200, 0, 0, 0)
        
        # State variables
        self.dobot_x, self.dobot_y, self.dobot_z, self.dobot_r = self.HOME_POSITION
        self.last_command_time = time.time()
        self.air_pump_on = False
        self.dobot = None
        self.dobot_connected = False
        
        # Arduino variables
        self.arduino = None
        self.arduino_connected = False
        self.motor_direction = "STOP"
        self.left_hand_detected = False
        self.hand_in_left_zone = False
        self.hand_in_right_zone = False
        
        # Control zone parameters
        self.left_zone_width_ratio = 0.3  # 30% of screen width
        self.zone_trigger_margin = 0.1    # 10% margin from edges
        
        # Initialize components
        self.setup_mediapipe()
        self.connect_arduino()
        self.connect_dobot()
        self.setup_camera()
        
        if self.arduino_connected:
            self.start_arduino_monitoring()
        
    def setup_mediapipe(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=2
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
    def find_arduino_port(self):
        ports = list_ports.comports()
        for port in ports:
            if any(desc in port.description.lower() for desc in ['arduino', 'ch340', 'cp210x', 'ftdi']):
                return port.device
        return None
        
    def connect_arduino(self):
        arduino_port = self.find_arduino_port()
        if arduino_port is None:
            print("Arduino not found - motor control disabled")
            return False
            
        try:
            self.arduino = serial.Serial(arduino_port, 9600, timeout=1)
            time.sleep(2)
            print(f"Arduino connected on {arduino_port}")
            self.arduino_connected = True
            return True
        except Exception as e:
            print(f"Arduino connection failed: {e}")
            return False
            
    def start_arduino_monitoring(self):
        def monitor_sensors():
            while self.arduino_connected:
                try:
                    if self.arduino.in_waiting > 0:
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
        
    def find_dobot_port(self):
        ports = list_ports.comports()
        for port in ports:
            if 'Silicon Labs CP210x USB to UART Bridge' in port.description:
                return port.device
        return None
        
    def connect_dobot(self):
        dobot_port = self.find_dobot_port()
        if dobot_port is None:
            print("Dobot not found - arm control disabled")
            return False
            
        try:
            self.dobot = Dobot(port=dobot_port)
            print(f"Dobot connected on {dobot_port}")
            self.dobot.home()
            self.dobot.move_to(*self.HOME_POSITION, wait=True)
            self.dobot_connected = True
            return True
        except Exception as e:
            print(f"Dobot connection failed: {e}")
            return False
            
    def send_motor_command(self, direction):
        if not self.arduino_connected:
            return
            
        try:
            if direction != self.motor_direction:
                command = f"MOTOR:{direction}\n"
                self.arduino.write(command.encode())
                self.motor_direction = direction
                print(f"Motor: {direction}")
        except Exception as e:
            print(f"Motor command error: {e}")
            
    def process_left_hand_motor_control(self, x, img_width):
        if not self.left_hand_detected:
            if self.motor_direction != "STOP":
                self.send_motor_command("STOP")
            return
            
        # Calculate zone boundaries
        left_zone_width = img_width * self.left_zone_width_ratio
        left_trigger = left_zone_width * self.zone_trigger_margin
        right_trigger = left_zone_width * (1 - self.zone_trigger_margin)
        
        # Only move when hand crosses the boundaries
        if x < left_trigger:
            if not self.hand_in_left_zone:
                self.send_motor_command("LEFT")
                self.hand_in_left_zone = True
                self.hand_in_right_zone = False
        elif x > right_trigger:
            if not self.hand_in_right_zone:
                self.send_motor_command("RIGHT")
                self.hand_in_right_zone = True
                self.hand_in_left_zone = False
        else:
            # Hand is in center zone
            if self.hand_in_left_zone or self.hand_in_right_zone:
                self.send_motor_command("STOP")
                self.hand_in_left_zone = False
                self.hand_in_right_zone = False
            
    def setup_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Camera not available")
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
    def process_hand_landmarks(self, hand_landmarks, img_width, img_height):
        # Get index finger tip position
        x = hand_landmarks.landmark[8].x * img_width
        y = hand_landmarks.landmark[8].y * img_height
        
        # Check for pinch gesture
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        distance = ((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2) ** 0.5
        is_pinched = distance < 0.05
        
        return x, y, is_pinched
        
    def map_hand_to_dobot(self, x, y, hand_label, img_width, img_height):
        if not self.dobot_connected:
            return
            
        if hand_label == "Right":
            # Map right hand to X and Y axes
            self.dobot_x = 100 + ((1 - (y / img_height)) * 200)  # X: 100-300
            x_center = img_width / 2
            x_relative = x - x_center
            self.dobot_y = (x_relative / x_center) * 200  # Y: -200 to 200
            
        elif hand_label == "Left":
            # Map left hand to Z axis (when not controlling motor)
            left_zone_width = img_width * self.left_zone_width_ratio
            if x > left_zone_width:  # Only control Z when outside motor zone
                self.dobot_z = -78 + ((1 - (y / img_height)) * 218)  # Z: -78 to 140
                
    def toggle_air_pump(self, state):
        if not self.dobot_connected:
            return
            
        try:
            self.dobot.suck(state)
            self.air_pump_on = state
            print(f"Air pump: {'ON' if state else 'OFF'}")
        except Exception as e:
            print(f"Air pump error: {e}")
            
    def draw_interface(self, frame):
        # Draw left hand control zone
        if self.left_hand_detected:
            img_height, img_width = frame.shape[:2]
            left_zone_width = int(img_width * self.left_zone_width_ratio)
            
            # Zone box
            cv2.rectangle(frame, (0, 0), (left_zone_width, img_height), (0, 255, 255), 2)
            
            # Trigger lines
            left_trigger = int(left_zone_width * self.zone_trigger_margin)
            right_trigger = int(left_zone_width * (1 - self.zone_trigger_margin))
            cv2.line(frame, (left_trigger, 0), (left_trigger, img_height), (255, 0, 0), 2)
            cv2.line(frame, (right_trigger, 0), (right_trigger, img_height), (255, 0, 0), 2)
            
            # Labels
            cv2.putText(frame, "MOTOR ZONE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Status info
        y_pos = 60
        if self.dobot_connected:
            cv2.putText(frame, f"Dobot: {int(self.dobot_x)}, {int(self.dobot_y)}, {int(self.dobot_z)}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_pos += 30
            cv2.putText(frame, f"Pump: {'ON' if self.air_pump_on else 'OFF'}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_pos += 30
        
        if self.arduino_connected:
            cv2.putText(frame, f"Motor: {self.motor_direction}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Controls
        cv2.putText(frame, "Right hand: X/Y control | Left hand: Z + Motor | Pinch: Pump | 'q': Quit", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                   
    def run(self):
        if not self.cap.isOpened():
            print("Camera not available")
            return
            
        print("Starting controller...")
        print(f"Dobot: {'Connected' if self.dobot_connected else 'Not connected'}")
        print(f"Arduino: {'Connected' if self.arduino_connected else 'Not connected'}")
        
        try:
            while self.cap.isOpened():
                success, frame = self.cap.read()
                if not success:
                    break
                    
                frame = cv2.flip(frame, 1)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_frame)
                
                self.left_hand_detected = False
                
                if results.multi_hand_landmarks:
                    for hand_landmarks, hand_handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                        hand_label = hand_handedness.classification[0].label
                        img_height, img_width, _ = frame.shape
                        
                        x, y, is_pinched = self.process_hand_landmarks(hand_landmarks, img_width, img_height)
                        
                        if hand_label == "Left":
                            self.left_hand_detected = True
                            left_zone_width = img_width * self.left_zone_width_ratio
                            if x <= left_zone_width:
                                self.process_left_hand_motor_control(x, img_width)
                        
                        self.map_hand_to_dobot(x, y, hand_label, img_width, img_height)
                        
                        # Toggle air pump on pinch
                        if is_pinched and time.time() - self.last_command_time > 1.0:
                            self.toggle_air_pump(not self.air_pump_on)
                            self.last_command_time = time.time()
                        
                        # Draw hand landmarks
                        self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Stop motor if no left hand
                if not self.left_hand_detected and self.motor_direction != "STOP":
                    self.send_motor_command("STOP")
                    self.hand_in_left_zone = False
                    self.hand_in_right_zone = False
                
                # Move Dobot
                if (self.dobot_connected and time.time() - self.last_command_time > self.COMMAND_DELAY):
                    try:
                        self.dobot.move_to(int(self.dobot_x), int(self.dobot_y), int(self.dobot_z), self.dobot_r, wait=False)
                    except Exception as e:
                        print(f"Dobot move error: {e}")
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
        
        if self.arduino_connected:
            self.send_motor_command("STOP")
            self.arduino.close()
        
        if self.dobot_connected:
            if self.air_pump_on:
                self.toggle_air_pump(False)
            try:
                self.dobot.move_to(*self.HOME_POSITION, wait=True)
            except:
                pass
            self.dobot.close()
        
        if self.cap.isOpened():
            self.cap.release()
            
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    controller = DobotHandController()
    controller.run()
