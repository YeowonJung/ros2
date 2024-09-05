import cv2
import numpy as np

# 이미지 읽기
img = cv2.imread('fire_image.jpg')

# 이미지에서 HSV로 변환
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 불꽃 색상 범위 설정 (예: 붉은 색상)
lower_bound = np.array([0, 50, 50])
upper_bound = np.array([10, 255, 255])

# 색상 범위 내의 영역을 마스크로 만듦
mask = cv2.inRange(hsv, lower_bound, upper_bound)

# 불꽃의 윤곽선 찾기
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# 윤곽선이 있을 경우, 중앙 좌표 계산
for contour in contours:
    if cv2.contourArea(contour) > 500:  # 작은 노이즈 제거
        x, y, w, h = cv2.boundingRect(contour)
        center_x = x + w // 2
        center_y = y + h // 2
        print(f"불꽃 중앙 좌표: x={center_x}, y={center_y}")
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

# 결과 이미지 출력
cv2.imshow('Fire Detection', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
