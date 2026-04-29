import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs
from smbus2 import SMBus
import time


class ObjectDetectionSystem:
    def __init__(self):
        self.direct = "none"
        self.direction = 0
        self.a = []
        self.max_speed = 80
        self.model = YOLO(r"/home/VSK/Desktop/best.pt")
        self.cap = None
        self.pipeline = None
        self.initialize_system()

    def initialize_system(self):
        """Initialize or reinitialize video capture and depth camera pipeline."""
        self.cap = cv2.VideoCapture(4)

        # Initialize depth camera pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Try starting the pipeline, and wait until it's connected
        while True:
            try:
                self.pipeline.start(config)
                print("Depth camera connected successfully.")
                break
            except RuntimeError:
                print("Error: No device connected. Waiting for camera...")
                time.sleep(2)  # Wait before retrying

    def process_frame(self, frame, depth_frame, x1, y1, x2, y2, cx, cy, cls, conf):
        """Process individual frame for object detection and related logic."""
        # Calculate angle and distance
        angle = self.find_angle(cx, cy, frame.shape[1])
        distance = self.calculate_distance_from_depth(depth_frame, cx, cy)

        # Display bounding box and information
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(frame, f"Class: {cls} | Conf: {conf:.2f} | Ang: {angle}",
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"Direction: {self.direct} | Dist: {distance}",
                    (x1, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 0), -1)

        return angle, distance

    def send_data(self, angle, direct, distance):
        """Send data to external devices using SMBus."""
        if direct == 'Right':
            direct_code = 1
        elif direct == 'Left':
            direct_code = 2
        else:
            direct_code = 0

        if distance < self.max_speed and distance > 200:
            distance = self.max_speed
        elif distance > self.max_speed:
            distance = self.max_speed
        else:
            distance = 0

        print(f"start")
        self.WN(int(angle), int(direct_code), int(distance))
        print(f"dn3{int(distance)}")

    def WN(self, angle123, direct123, distance1234):
        from smbus2 import SMBus
        import time

        bus = SMBus(1)  # Use I2C bus 1 (default on Raspberry Pi)
        address = 0x40  # I2C address of the device

        try:
            # Ensure the values are within the byte range (0-255)
            angle123 = max(0, min(255, angle123))
            direct123 = max(0, min(255, direct123))
            distance1234 = max(0, min(255, distance1234))

            # Create a 3-byte array
            byte_array = [angle123, direct123, distance1234]

            # Write the byte array to the device
            bus.write_i2c_block_data(address, 0x00, byte_array)  # 0x00 can be the register/command
            time.sleep(0.1)  # Short delay for reliability
            print(f"Sent byte array: {byte_array}")
        except IOError as e:
            print(f"I2C Error: {e}")
        finally:
            bus.close()

    def map_value(self, value, x, y, ax, b):
        return ax + (value - x) * (b - ax) / (y - x)

    def find_angle(self,x, y, frame_width):
        global direct
        centerX = frame_width // 2
        diff = x - centerX
        return int(self.map_value(x, 0, frame_width, 0, 70))


    def calculate_distance_from_depth(self, depth_frame, cx, cy):
        """Calculate distance of an object from the depth frame."""
        distance = depth_frame.get_distance(cx, cy)
        return distance

    def detect_arrow_direction_from_video(self,depth_frame123, frame, cx, cy,X1,X2,Y1,Y2):
        global a, direct, direction
        a = []


        # Initialize 'direct' with a default value
        direct = "None"
        direct123 = "None"
        tilt=0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 70, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("arrow Frame", frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 7 and cv2.contourArea(contour) > 100:
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                arrowhead = max(approx, key=lambda p: np.linalg.norm((p[0][0] - cx, p[0][1] - cy)))

                direction = 1 if arrowhead[0][0] < cx else 2 if arrowhead[0][0] > cx else 3
                if len(a) <= 1:
                    a.append(direction)
                else:
                    a.pop(0)
                    a.append(direction)

                right = a.count(1)
                left = a.count(2)
                none = a.count(3)

                if right >= 1:
                    direct = "Right"
                    # return direct
                elif left >= 1:
                    direct = "Left"
                    # return direct
                else:
                    direct = "None"

                diff_distance1 = depth_frame123.get_distance(int(X1), int(Y1))
                diff_distance2 = depth_frame123.get_distance(int(X2), int(Y1))
                diff_distance = diff_distance1 - diff_distance2
                diff_distance = abs(diff_distance)
                if diff_distance > (0.1 * diff_distance1):
                    print("tiltedddd")
                    if diff_distance1 > diff_distance2:

                        tilt = (X2 - X1) // diff_distance
                    elif diff_distance2 > diff_distance1:
                        tilt = -((X2 - X1) // diff_distance)
                else:
                    tilt = 0

                if direct == 'Right':
                    direct123 = -90 - tilt
                    if direct123>145 :
                        direct123 = 145
                    if direct123<-145:
                        direct123 = -145

                elif direct == 'Left':
                    direct123 = 90 -tilt
                    if direct123>145 :
                        direct123 = 145
                    if direct123<-145:
                        direct123 = -145
                else :
                    direct123 = 0
                direct123=int(self.map_value(direct123, -145,145,0,255))
        return direct123

    # def readFrame(self):
    #     for _ in range(5):  # Discard outdated frames
    #         ret, frame = self.cap.read()
    #         if not ret:
    #             print("Error: Could not read frame. Trying to reinitialize camera...")
    #             self.reinitialize_camera()
    #             continue
    #     return frame

    def run(self):
        try:
            with open("/home/samiksha/startup_log.txt", "a") as log_file:
                log_file.write("Script executed at startup\n")
                print("Code started and log file updated.")

            while True:
                # ret, frame = self.cap.read()
                # if not ret:
                #     print("Error: Could not read frame. Trying to reinitialize camera...")
                #     self.reinitialize_camera()
                #     continue

                for _ in range(5):  # Discard outdated frames
                    ret, frame = self.cap.read()
                    if not ret:
                        print("Error: Could not read frame. Trying to reinitialize camera...")
                        self.reinitialize_camera()
                        continue

                frames = None
                while not frames:
                    try:
                        frames = self.pipeline.wait_for_frames()
                        depth_frame = frames.get_depth_frame()
                        if not depth_frame:
                            raise RuntimeError("No depth frame received.")
                    except Exception as e:
                        print(f"Error: {e}. Waiting for camera to reconnect...")
                        time.sleep(1)  # Wait before retrying
                        self.reinitialize_camera()  # Reinitialize camera on failure

                depth_image = np.asanyarray(depth_frame.get_data())
                depth_frame2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                results = self.model.predict(frame, verbose=True)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = x1 + (x2 - x1) // 2
                        cy = y1 + (y2 - y1) // 2

                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        if conf > 0.4:
                            angle, distance = self.process_frame(frame, depth_frame, x1, y1, x2, y2, cx, cy, cls, conf)
                            print("distance:" , distance)
                            labl = ["Right","Left","cone"]
                            # self.detect_arrow_direction_from_video(frame, cx, cy)
                            # self.direct = labl[int(box.cls[0])]
                            if cls=='Right' or cls=='Left':
                                self.direct = self.detect_arrow_direction_from_video(depth_frame,frame[y1:y2, x1:x2], cx, cy,x1,x2,y1,y2)
                            elif cls=='cone':
                                angle=200
                                self.direct=0
                            try:
                                distance = distance*100
                                self.send_data(angle, self.direct, distance)
                            except Exception as e:
                                print(f"i2c error:{e}")
                                continue

                cv2.imshow("YOLO Arrow Detection", frame)
                cv2.imshow("Depth Frame", depth_frame2)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

    def reinitialize_camera(self):
        """Reinitialize video capture and depth camera pipeline."""
        self.cleanup()
        print("Reinitializing camera...")
        time.sleep(2)  # Give time for reinitialization
        self.initialize_system()

    def cleanup(self):
        """Cleanup resources."""
        if self.cap:
            self.cap.release()
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()



if __name__ == "__main__":
    system = ObjectDetectionSystem()
    system.run()
