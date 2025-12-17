import cv2
import numpy as np
import threading
import time
import socket
import struct
import asyncio

#run this in terminal first
#switch to C:, then --> cd %LOCALAPPDATA%\Android\Sdk\platform-tools
#run adb forward tcp:6001 tcp:6001

# --- CONFIGURATION ---
HOST = '127.0.0.1'  # Localhost (via ADB forwarding)
PORT = 6001         # The Video Port
CONNECTION_RETRY_DELAY = 2 # Seconds to wait before retrying connection

# --- SHARED STATE ---
last_frame = None
frame_lock = threading.Lock()
frame_fetch_running = False
camera_connected = False  # Replaces 'cameraHealth'

vision_state = {
    "found": False,
    "x": 0,      # Center X
    "y": 0,      # Center Y
    "w": 0,      # Width (for bounding box)
    "h": 0,      # Height (for bounding box)
    "cx": 0,
    "cy": 0,
}

COLOR_RANGES = {
    "RED1":   ((0, 140, 70),   (8, 255, 255)),
    "RED2":   ((172, 140, 70), (180, 255, 255)),
    "BLUE":   ((100,120, 70),  (140,255,255)),
    "YELLOW": ((20,100,100),   (35,255,255)),
}

# --- HELPER: Receive exact number of bytes ---
def recvall(sock, n):
    """Helper to ensure we read exactly n bytes from TCP stream."""
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def _frame_fetch_loop():
    global last_frame, frame_fetch_running, camera_connected

    print(f"[Camera] connecting to Android at {HOST}:{PORT}...")
    frame_fetch_running = True

    while frame_fetch_running:
        s = None
        try:
            # 1. Establish Connection
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0) # Timeout if phone stops sending
            s.connect((HOST, PORT))
            print("[Camera] Connected to Android Video Stream!")
            camera_connected = True

            # 2. Continuous Read Loop
            while frame_fetch_running:
                # A. Read Image Size (4 bytes int)
                size_data = recvall(s, 4)
                if not size_data:
                    raise socket.error("Remote closed connection (0 bytes read)")
                
                # Unpack big-endian unsigned int (>I)
                size = struct.unpack('>I', size_data)[0]

                # B. Read Image Data
                img_data = recvall(s, size)
                if not img_data:
                    raise socket.error("Failed to read image data")

                # C. Decode
                np_arr = np.frombuffer(img_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    with frame_lock:
                        last_frame = frame
                
        except (socket.error, socket.timeout) as e:
            print(f"[Camera] Connection lost/failed: {e}. Retrying in {CONNECTION_RETRY_DELAY}s...")
            camera_connected = False
            with frame_lock:
                last_frame = None # Clear stale frame
            time.sleep(CONNECTION_RETRY_DELAY)
            
        finally:
            if s: 
                s.close()
            camera_connected = False

    print("[Camera] Frame fetcher stopped")
    
def start_frame_fetcher():
    """Starts the TCP client thread."""
    global frame_fetch_running

    if frame_fetch_running:
        return
    t = threading.Thread(target=_frame_fetch_loop, daemon=True)
    t.start()

# Auto-start the fetcher when module loads
start_frame_fetcher()

def checkConnection():
    """Returns True if Android video socket is active."""
    return camera_connected

async def detect_target_color(target):
    """
    Detects color in the latest frame from Android.
    Returns: (bool found, int cx, int cy)
    """
    global last_frame, vision_state

    # 1. Get the frame safely
    with frame_lock:
        if last_frame is None:
            return False, None, None
        frame = last_frame.copy()

    # 2. Process frame (Same logic as before)
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if target == "RED":
        low1, high1 = COLOR_RANGES["RED1"]
        low2, high2 = COLOR_RANGES["RED2"]
        mask1 = cv2.inRange(hsv, low1, high1)
        mask2 = cv2.inRange(hsv, low2, high2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        # Default logic for Yellow, Blue, etc.
        if target not in COLOR_RANGES:
            print(f"[Camera] Unknown target color: {target}")
            return False, None, None
            
        low, high = COLOR_RANGES[target]
        mask = cv2.inRange(hsv, low, high)

    # 3. Find Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False, None, None

    # 4. Get Largest Blob
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 500: # Threshold for noise
        return False, None, None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return False, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x, y, w, h = cv2.boundingRect(c)
    
    vision_state["x"] = x
    vision_state["y"] = y
    vision_state["w"] = w
    vision_state["h"] = h
    vision_state["cx"] = cx
    vision_state["cy"] = cy
    
    return True, cx, cy

# --- STREAM WINDOW (Optional Debugging) ---
stream_enabled = False

def _stream_loop():
    global stream_enabled, vision_state

    print("[Camera] Stream window started")
    
    while stream_enabled:
        with frame_lock:
            if last_frame is None:
                frame = None
            else:
                frame = last_frame.copy()

        if frame is not None:
            # Resize for easier viewing on laptop screen
            display_frame = cv2.resize(frame, (640, 480))
            
            # Optional: Add text status
            status_color = (0, 255, 0) if camera_connected else (0, 0, 255)
            cv2.circle(display_frame, (30, 30), 10, status_color, -1) 
            
            cv2.rectangle(display_frame, (vision_state["x"], vision_state["y"]), (vision_state["x"] + vision_state["w"], vision_state["y"] + vision_state["h"]), (0, 255, 0), 2)
            cv2.circle(display_frame, (vision_state["cx"], vision_state["cy"]), 5, (0, 0, 255), -1)
            
            text = f"{vision_state["cx"]},{vision_state["cy"]}"
            cv2.putText(display_frame, text, (vision_state["x"], vision_state["y"] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Android USB Stream", display_frame)
        else:
            # If no frame, just sleep a bit
            pass

        if cv2.waitKey(1) == 27: # ESC to close
            break

        time.sleep(0.03) # ~30 FPS

    cv2.destroyAllWindows()
    stream_enabled = False
    print("[Camera] Stream window closed")

def start_stream():
    """Show the stream window."""
    global stream_enabled
    if stream_enabled: return
    stream_enabled = True
    t = threading.Thread(target=_stream_loop, daemon=True)
    t.start()

def stop_stream():
    global stream_enabled
    stream_enabled = False