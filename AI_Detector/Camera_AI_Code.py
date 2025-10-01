import cv2
import time
import threading
from queue import Queue, Empty
from ultralytics import YOLO

# ================== CẤU HÌNH ==================
# Dùng webcam (0 = camera mặc định, 1 = camera ngoài nếu có)
CAMERA_SOURCE = 0             # <--- đổi sang số hoặc RTSP URL nếu muốn
MODEL_PATH = 'car.pt'
IMGSZ = 640
CONF_THRES = 0.5
ROI = (150, 150, 500, 400)    # (x1, y1, x2, y2) - khung hình chữ nhật cần giám sát
QUEUE_MAXSIZE = 4             # giới hạn hàng đợi để tránh dồn bộ nhớ
# ==============================================

def corner_inside_roi(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
    """Trả về True nếu ít nhất 1 góc bbox nằm trong ROI."""
    for (cx, cy) in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
        if rx1 <= cx <= rx2 and ry1 <= cy <= ry2:
            return True
    return False

def camera_reader(stop_event: threading.Event, frame_q: Queue, source, imsz: int):
    """Thread 1: đọc camera (webcam hoặc RTSP) và đẩy frame đã resize vào queue."""
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print("[ERROR] Không mở được camera.")
        stop_event.set()
        return

    print("[READER] Bắt đầu đọc camera...")
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
    print("[READER] Dừng reader.")

def detector_worker(stop_event: threading.Event, frame_q: Queue, model_path: str, imsz: int, conf_thres: float, roi):
    """Thread 2: lấy frame từ queue, chạy YOLO, kiểm tra ROI, vẽ và hiển thị."""
    print("[WORKER] Đang load model:", model_path)
    model = YOLO(model_path)

    # Tìm class id cho "car"
    names = model.names if hasattr(model, "names") else {}
    if isinstance(names, dict):
        car_ids = [i for i, n in names.items() if str(n).lower() == "car"]
    else:
        car_ids = []
    if not car_ids:
        car_ids = [0]

    print(f"[WORKER] Car class IDs: {car_ids}")

    rx1, ry1, rx2, ry2 = roi
    win_name = "Car in ROI - 2 Threads"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    while not stop_event.is_set():
        try:
            frame = frame_q.get(timeout=0.1)
        except Empty:
            continue

        # Inference
        results = model(frame, imgsz=imsz, conf=conf_thres, verbose=False)

        # Vẽ ROI
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 0, 255), 2)
        cv2.putText(frame, "ROI", (rx1, max(0, ry1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        alert = False
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls not in car_ids:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"car {conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                if corner_inside_roi(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
                    alert = True

        if alert:
            cv2.putText(frame, "ALERT!", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow(win_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cv2.destroyAllWindows()
    print("[WORKER] Dừng worker.")

def main():
    frame_q = Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = threading.Event()

    reader_t = threading.Thread(target=camera_reader, args=(stop_event, frame_q, CAMERA_SOURCE, IMGSZ), daemon=True)
    worker_t = threading.Thread(target=detector_worker, args=(stop_event, frame_q, MODEL_PATH, IMGSZ, CONF_THRES, ROI), daemon=True)

    reader_t.start()
    worker_t.start()

    try:
        while worker_t.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    reader_t.join(timeout=1.0)
    worker_t.join(timeout=1.0)
    print("[MAIN] Done.")

if __name__ == "__main__":
    main()
