import cv2
import time
import threading
from queue import Queue, Empty
from ultralytics import YOLO
from playsound import playsound

# ================== CẤU HÌNH ==================
RTSP_URL = 'rtsp://admin:EEZSQY@192.168.1.109:554/cam/realmonitor?channel=1&subtype=0'
MODEL_PATH = 'car.pt'
IMGSZ = 640
CONF_THRES = 0.5
ROI = (150, 150, 500, 400)
QUEUE_MAXSIZE = 4
ALERT_SOUND = "baodong.mp3"
# ==============================================

def corner_inside_roi(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
    for (cx, cy) in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
        if rx1 <= cx <= rx2 and ry1 <= cy <= ry2:
            return True
    return False

def camera_reader(stop_event, frame_q, rtsp_url, imsz):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("[ERROR] Không mở được camera.")
        stop_event.set()
        return
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue
        frame = cv2.resize(frame, (imsz, imsz))
        if frame_q.full():
            try:
                frame_q.get_nowait()
            except Empty:
                pass
        frame_q.put(frame, timeout=0.01)
    cap.release()

def play_alert_sound():
    """Chạy playsound trong thread riêng để không block video"""
    threading.Thread(target=playsound, args=(ALERT_SOUND,), daemon=True).start()

def detector_worker(stop_event, frame_q, model_path, imsz, conf_thres, roi):
    model = YOLO(model_path)
    names = model.names if hasattr(model, "names") else {}
    car_ids = [i for i, n in names.items() if str(n).lower() == "car"] or [0]

    rx1, ry1, rx2, ry2 = roi
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)

    last_alert = False

    while not stop_event.is_set():
        try:
            frame = frame_q.get(timeout=0.1)
        except Empty:
            continue

        results = model(frame, imgsz=imsz, conf=conf_thres, verbose=False)
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 0, 255), 2)
        cv2.putText(frame, "ROI", (rx1, ry1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        alert = False
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) not in car_ids:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                if corner_inside_roi(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
                    alert = True

        if alert:
            cv2.putText(frame, "ALERT!", (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
            if not last_alert:  # chỉ phát khi chuyển từ an toàn sang cảnh báo
                play_alert_sound()
        last_alert = alert

        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            stop_event.set()
            break

    cv2.destroyAllWindows()

def main():
    frame_q = Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = threading.Event()
    t1 = threading.Thread(target=camera_reader, args=(stop_event, frame_q, RTSP_URL, IMGSZ), daemon=True)
    t2 = threading.Thread(target=detector_worker, args=(stop_event, frame_q, MODEL_PATH, IMGSZ, CONF_THRES, ROI), daemon=True)
    t1.start(); t2.start()
    try:
        while t2.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()
    t1.join(); t2.join()

if __name__ == "__main__":
    main()
