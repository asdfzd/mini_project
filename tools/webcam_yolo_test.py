import cv2
import sys
from pathlib import Path
from ultralytics import YOLO

camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

model_path = str(Path(__file__).resolve().parents[1] / "models" / "my_best_v2.pt")
target_class_id = 1  # 기존 코드 기준: 0=box, 1=car

if not Path(model_path).exists():
    print(f"[ERROR] 모델 파일 없음: {model_path}")
    sys.exit(1)

model = YOLO(model_path)

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print(f"[ERROR] camera_index={camera_index} 카메라를 열 수 없습니다.")
    sys.exit(1)

print(f"[OK] camera_index={camera_index} 카메라 열림")
print(f"[OK] YOLO model: {model_path}")
print(f"[INFO] target_class_id={target_class_id}")
print("q 누르면 종료")

while True:
    ret, frame = cap.read()

    if not ret:
        print("[ERROR] 프레임 읽기 실패")
        break

    results = model(frame, verbose=False)
    boxes = results[0].boxes

    detected = []

    for b in boxes:
        cls_id = int(b.cls[0])
        conf = float(b.conf[0])
        x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())

        detected.append((cls_id, round(conf, 2)))

        label = f"class={cls_id}, conf={conf:.2f}"

        if cls_id == target_class_id:
            color = (0, 255, 0)
            label = f"TARGET car? class={cls_id}, conf={conf:.2f}"
        else:
            color = (0, 0, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    if detected:
        print(f"[YOLO] detected={detected}")

    cv2.imshow(f"YOLO Webcam Test index={camera_index}", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
