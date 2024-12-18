import cv2
import numpy as np
import time as t
import pyrealsense2 as rs

a = []
direct = "none"
direction = 0
depth_of_arrow = 0


def detect_arrow_direction_from_video():
    global a, direct, direction, depth_of_arrow, flag, dir_scalar
    i2, i3 = 0, 0
    ptime, ctime = 0, 0
    flag = False
    counter=0
    dir_scalar=0   # Depth camera setup
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable both color (RGB) and depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start the pipeline
    pipeline.start(config)

    while True:
        try:
            # Wait for frames
            frames = pipeline.wait_for_frames()

            # Get both color and depth frames
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("Frames not available")
                continue

            # Convert frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
        except Exception as e:
            print(f"Error capturing frames: {e}")
            continue

        if depth_image is None or color_image is None:
            print("No frame detected.")
            continue

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 70, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        arrow_detected = False

        depth_of_arrow = 0

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 7 and cv2.contourArea(contour) > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h

                if 0.7 < aspect_ratio < 2.0:
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

                    cv2.drawContours(depth_colormap, [approx], 0, (0, 255, 0), 2)
                    cv2.circle(depth_colormap, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.drawContours(color_image, [approx], 0, (0, 255, 0), 2)
                    cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
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
                        flag=True
                        conunter =1
                        # 20x20 area around centroid
                        depth_region = depth_image[cy - 10:cy + 10, cx - 10:cx + 10]
                        if depth_region.size > 0:
                            depth_of_arrow = np.mean(depth_region)
                            depth_of_arrow = int(depth_of_arrow / 100)
                        direct = "Right"
                        dir_scalar=1
                        print(f"Arrow detected pointing: Right")


                    elif left >= 25:
                        flag=True
                        conunter =2

                        # 20x20 area around centroid
                        depth_region = depth_image[cy - 10:cy + 10, cx - 10:cx + 10]

                        if depth_region.size > 0:

                            depth_of_arrow = np.mean(depth_region)
                            depth_of_arrow = int(depth_of_arrow / 100)
                        direct = "Left"
                        dir_scalar=2
                        print(f"Arrow detected pointing: Left")


                    elif none >= 25:
                        direct = "None"
                        dir_scalar=0
                        print(f"Arrow detected pointing: None")

                    if direct == "Right":
                        dir_scalar=1
                    elif direct == "Left":
                        dir_scalar=2

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

        # Display info

        cv2.putText(color_image, direct, (50, 120), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(color_image, str(depth_of_arrow), (50, 75), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2,cv2.LINE_AA)
        cv2.putText(color_image, str(round(fps)), (50, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2, cv2.LINE_AA)

        # Display both the RGB and depth feeds
        cv2.imshow('RGB Feed', color_image)
        cv2.imshow('Depth Feed', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('a'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    detect_arrow_direction_from_video()
