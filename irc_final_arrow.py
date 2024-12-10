import cv2
import numpy as np
import time as t

a = []
direct = "none"
direction = 0

def detect_arrow_direction_from_video():
    global a, direct, direction
    i2, i3 = 0, 0
    ptime, ctime = 0, 0

    getvideo = cv2.VideoCapture(0)

    if not getvideo.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = getvideo.read()
        if not ret:
            print("No frames detected.")
            continue

        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 70, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"contours: {len(contours)}")

        arrow_detected = False

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 7 and cv2.contourArea(contour) > 2000:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h

                if 1.3 < aspect_ratio < 1.7:
                    M = cv2.moments(contour)
                    if M["m00"] == 0:
                        continue

                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    arrowhead = max(approx, key=lambda p: np.linalg.norm((p[0][0] - cx, p[0][1] - cy)))

                    if arrowhead[0][0] < cx:
                        direction = 1
                    elif arrowhead[0][0] > cx:
                        direction = 2
                    else:
                        direction = 3

                    cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    print(f"Arrow detected pointing: {direction}")
                    arrow_detected = True

                    if len(a) < 30:
                        a.append(direction)
                    else:
                        a.pop(0)
                        a.append(direction)

                    right = a.count(1)
                    left = a.count(2)
                    none = a.count(3)

                    if right >= 25:
                        direct = "Right"
                        print(f"Arrow detected pointing: Right")
                    elif left >= 25:
                        direct = "Left"
                        print(f"Arrow detected pointing: Left")
                    elif none >= 25:
                        direct = "None"
                        print(f"Arrow detected pointing: None")

        if not arrow_detected:
            i3 += 1
            print(i3)
            if i3 > 10:
                direction = 3
                i3 = 0
                if len(a) < 30:
                    a.append(direction)
                else:
                    a.pop(0)
                    a.append(direction)

            if a.count(3) > 25:
                direct = "None"
            print("No arrow detected.")

        fps = 1 / (t.time() - ptime)
        ptime = t.time()

        cv2.putText(frame, direct, (50, 120), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, str(round(fps)), (50, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2, cv2.LINE_AA)
        print(f"FPS: {fps}")
        cv2.imshow('Arrow Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('a'):
            break

    getvideo.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_arrow_direction_from_video()
