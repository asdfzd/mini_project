import cv2
import sys

# 실행할 때 번호를 안 주면 0번 카메라 사용
camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print(f"[ERROR] camera_index={camera_index} 카메라를 열 수 없습니다.")
    sys.exit(1)

print(f"[OK] camera_index={camera_index} 카메라 열림")
print("화면 창에서 q 누르면 종료")

while True:
    ret, frame = cap.read()

    if not ret:
        print("[ERROR] 프레임을 읽지 못했습니다.")
        break

    cv2.imshow(f"Camera index {camera_index}", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

