
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

align_to = rs.stream.color
align = rs.align(align_to)

# create save path
save_path = os.path.join(os.getcwd(), "out", time.strftime(
    "%Y_%m_%d_%H_%M_%S", time.localtime()))
parent_dir = os.path.dirname(save_path)
os.makedirs(parent_dir, exist_ok=True)
os.mkdir(save_path)
os.mkdir(os.path.join(save_path, "color"))
os.mkdir(os.path.join(save_path, "depth"))


cv2.namedWindow("live", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("save", cv2.WINDOW_AUTOSIZE)
saved_color_image = None
saved_depth_mapped_image = None
saved_count = 0

try:
    while True:
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        depth_data = np.asanyarray(
            aligned_depth_frame.get_data(), dtype="float16")
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_mapped_image = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow("live", np.hstack((color_image, depth_mapped_image)))
        key = cv2.waitKey(30)

        # s: save
        if key & 0xFF == ord('s'):
            saved_color_image = color_image
            saved_depth_mapped_image = depth_mapped_image

            cv2.imwrite(os.path.join((save_path), "color",
                        "{}.png".format(saved_count)), saved_color_image)
            np.save(os.path.join((save_path), "depth",
                    "{}".format(saved_count)), depth_data)
            saved_count += 1
            cv2.imshow("save", np.hstack(
                (saved_color_image, saved_depth_mapped_image)))

        # q:quit
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
