import cv2
import numpy as np
import requests
import threading
import time
ESP32_URL = "http://10.222.168.44/cam.jpg"

last_frame = None
frame_lock = threading.Lock()
frame_fetch_running = False

COLOR_RANGES = {
    "RED1":   ((0, 120, 70),   (10, 255, 255)),
    "RED2":   ((170, 120, 70), (180, 255, 255)),
    "BLUE":   ((100,120, 70),  (140,255,255)),
    "YELLOW": ((20,100,100),   (35,255,255)),
}

def _frame_fetch_loop():
    global last_frame, frame_fetch_running

    print("[Camera] Frame fetcher started")
    frame_fetch_running = True

    while frame_fetch_running:
        try:
            r = requests.get(ESP32_URL, timeout=0.5)
            img = np.frombuffer(r.content, np.uint8)
            frame = cv2.imdecode(img, cv2.IMREAD_COLOR)

            if frame is not None:
                with frame_lock:
                    last_frame = frame

        except Exception as e:
            with frame_lock:
                last_frame = None

        time.sleep(0.05)   # limit to ~20 FPS (safe for ESP32)

    print("[Camera] Frame fetcher stopped")
    
def start_frame_fetcher():
    """Starts the camera frame-fetching thread."""
    global frame_fetch_running

    if frame_fetch_running:
        return

    t = threading.Thread(target=_frame_fetch_loop, daemon=True)
    t.start()
    
start_frame_fetcher()

def checkConnection():
    """
    Returns True if ESP32 camera is reachable.
    Returns False if request fails.
    """
    global cameraHealth
    if cameraHealth == True:
        cameraHealth = True
        return True
    else:
        try:
            r = requests.head(ESP32_URL, timeout=0.2)
            if r.status_code == 200:
                cameraHealth = True
                return True
            else:
                return False
        except:
            return False
        
    

async def detect_target_color(target):

    # If camera has failed in the past → always fail now
    # if cameraHealth is False:
    #     return False, None, None

    # First-time check OR expected to work
    global last_frame

    # If no frame available → camera offline or slow
    with frame_lock:
        frame = None if last_frame is None else last_frame.copy()

    if frame is None:
        return False, None, None

    # Process frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if target == "RED":
        low1, high1 = COLOR_RANGES["RED1"]
        low2, high2 = COLOR_RANGES["RED2"]

        mask1 = cv2.inRange(hsv, low1, high1)
        mask2 = cv2.inRange(hsv, low2, high2)

        mask = cv2.bitwise_or(mask1, mask2)

    else:
        low, high = COLOR_RANGES[target]
        mask = cv2.inRange(hsv, low, high)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False, None, None

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:
        return False, None, None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return False, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return True, cx, cy

stream_enabled = False
stream_thread_running = False

def _stream_loop():
    global stream_thread_running

    print("[Camera] Stream window started")
    stream_thread_running = True

    while stream_enabled:
        with frame_lock:
            frame = None if last_frame is None else last_frame.copy()

        if frame is None:
            continue
        else:
            frame = cv2.resize(frame, (640, 480))
            cv2.imshow("ESP32 Stream", frame)

        if cv2.waitKey(1) == 27:
            break

        time.sleep(0.03)

    cv2.destroyAllWindows()
    stream_thread_running = False
    print("[Camera] Stream window closed")


def start_stream():
    """Show the ESP32 stream window."""
    global stream_enabled

    if stream_enabled:
        return

    stream_enabled = True
    t = threading.Thread(target=_stream_loop, daemon=True)
    t.start()


def stop_stream():
    """Stop the ESP32 stream window."""
    global stream_enabled
    stream_enabled = False


# print("[CameraColor] module ID =", id(sys.modules[__name__]))
# if __name__ == "__main__":
#     async def test():
#         print("Starting CameraColor test…")

#         ensure_camera_task()  # starts health monitor thread
#         print("MAIN module ID =", id(sys.modules['CameraColor']))
#         while True:
#             print(f"[Health] cameraHealth = {cameraHealth}")

#             detected, cx, cy = await detect_target_color("RED")

#             if cameraHealth:
#                 if detected:
#                     print(f"[Detect] RED detected at ({cx}, {cy})")
#                 else:
#                     print("[Detect] RED not found")
#             else:
#                 print("[Detect] Camera offline → skipping detection")

#             await asyncio.sleep(1.0)

#     asyncio.run(test())