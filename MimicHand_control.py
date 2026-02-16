import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# 1. Communication Setup
arduino_port = "COM3" 
try:
    arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=.01)
    time.sleep(2)
    print("Arduino ready!")
except Exception as e:
    print(f"Arduino connection skipped: {e}")

# 2. Smoothing and Filtering Parameters
last_percentages = [0, 0, 0, 0, 0] 
smoothing_factor = 0.22 
last_send_time = 0
send_delay = 0.05 

# MediaPipe Initialization
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

# Drawing Specifications
white_spec = mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=3, circle_radius=4)
connection_spec = mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2)

def map_finger(percentage, finger_name):
    """Maps normalized finger movement to pulse widths for specific servos."""
    limits = {
        "index":  {"min": 150, "max": 600, "reversed": True},
        "middle": {"min": 130, "max": 600, "reversed": False},
        "ring":   {"min": 150, "max": 600, "reversed": True},
        "pinky":  {"min": 170, "max": 520, "reversed": False},
        "thumb":  {"min": 170, "max": 520, "reversed": False}
    }
    p = limits[finger_name]
    val = np.interp(percentage, [0, 100], [p["max"], p["min"]]) if p["reversed"] else np.interp(percentage, [0, 100], [p["min"], p["max"]])
    return int(val)

def send_to_arduino(percentages):
    """Smooths finger data and sends encoded bytes to the microcontroller."""
    global last_send_time, last_percentages
    if (time.time() - last_send_time) < send_delay: return
    
    packet = bytearray()
    finger_names = ["index", "middle", "ring", "pinky", "thumb"]
    
    for i in range(5):
        # Apply Low-Pass Filter (Exponential Smoothing)
        smooth_percentage = (percentages[i] * smoothing_factor) + (last_percentages[i] * (1 - smoothing_factor))
        last_percentages[i] = smooth_percentage
        
        pulse = map_finger(smooth_percentage, finger_names[i])
        
        # Split 16-bit pulse value into two 8-bit bytes
        packet.append((pulse >> 8) & 0xFF)
        packet.append(pulse & 0xFF)
        
    arduino.write(packet)
    last_send_time = time.time()

# 4. Video Processing Loop
video_path = r"your_video_path.mp4"
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)

while cap.isOpened():
    start_time = time.time()
    success, frame = cap.read()
    if not success: break

    # Convert BGR to RGB for MediaPipe processing
    results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    h, w, _ = frame.shape
    canvas = np.zeros((h, w, 3), dtype=np.uint8) 

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(canvas, hand_landmarks, mp_hands.HAND_CONNECTIONS, white_spec, connection_spec)
            
            lm = hand_landmarks.landmark
            def get_dist(p1, p2): 
                return np.sqrt((lm[p1].x - lm[p2].x)**2 + (lm[p1].y - lm[p2].y)**2)
            
            # Use distance between wrist (0) and middle finger MCP (9) as reference for normalization
            ref_dist = get_dist(0, 9) 

            # --- SENSOR SYNCHRONIZATION AND CALIBRATION ---
            
            # Index: Adjusted lower bound to 0.6 and upper bound to 1.8 for optimal range.
            index = np.interp(get_dist(8, 0) / ref_dist, [0.6, 1.8], [100, 0])
            
            # Middle: Adjusted lower bound to 0.6 to match ring finger behavior.
            middle = np.interp(get_dist(12, 0) / ref_dist, [0.6, 1.9], [100, 0])
            
            # Ring: Reference calibration (optimized).
            ring = np.interp(get_dist(16, 0) / ref_dist, [0.4, 1.3], [100, 0])
            
            # Pinky: Adjusted lower bound to 0.5 for synchronization.
            pinky = np.interp(get_dist(20, 0) / ref_dist, [0.5, 2.0], [100, 0])
            
            # Thumb: Calibration set to [0.60, 1.3] for stability.
            thumb_ratio = get_dist(4, 9) / ref_dist
            thumb = np.interp(thumb_ratio, [0.60, 1.3], [100, 0])

            send_to_arduino([index, middle, ring, pinky, thumb])

    cv2.imshow('Raw Video', frame)
    cv2.imshow('Landmark Analysis', canvas)

    # Frame rate control
    elapsed = (time.time() - start_time) * 1000
    wait_ms = max(1, int((1000/fps) - elapsed))
    if cv2.waitKey(wait_ms) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()