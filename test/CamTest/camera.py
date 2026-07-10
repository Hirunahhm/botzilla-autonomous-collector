import numpy as np
import cv2
import time
import freenect

print("Starting camera")
capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
print("Captured camera")

prev_frame_time = 0
new_frame_time = 0

frame_layout_top = 240
frame_layout_sides = 200

def find_contours(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # gray = gray[height // 2 :, :]  # Crop the lower half of the image
    # frame = frame[height // 2 :, :]  # Crop the lower half of the image

    # Apply Gaussian blur to reduce noise
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply a threshold to create a binary image
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    # canimg = cv2.Canny(thresh, 50, 200)

    # kernel = np.ones((3,3), np.uint8)
    # thresh = cv2.erode(thresh, kernel, iterations=5)
    # thresh = cv2.dilate(thresh, kernel, iterations=9)

    # Find contours in the binary image
    return cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

while True:
    frame, timestamp = freenect.sync_get_video()
    new_frame_time = time.time()

    fps = 1 / (new_frame_time - prev_frame_time)
    height, width, _ = frame.shape

    main_frame = frame[frame_layout_top:height, frame_layout_sides:width-frame_layout_sides]
    top_frame = frame[:frame_layout_top, frame_layout_sides:width-frame_layout_sides]
    left_frame = frame[frame_layout_top:, :frame_layout_sides]
    right_frame = frame[frame_layout_top:, width-frame_layout_sides:]

    cv2.rectangle(frame, (frame_layout_sides, frame_layout_top), (width-frame_layout_sides, height), (255, 255, 255), 2)

    contours, hierarchy = find_contours(main_frame)
    left_contours, _ = find_contours(left_frame)
    right_contours, _ = find_contours(right_frame)
    top_contours, _ = find_contours(top_frame)

    # Main line following logic
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest_contour)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Determine whether to turn left or right based on the line position (centered or not)
            if cx < frame.shape[1] // 2:
                # Turn left
                # my_kobuki.move(100, 255, 0)
                print("Turn left")
            else:
                # Turn right
                # my_kobuki.move(255, 100, 0)
                print("Turn right")

    is_junction = ((len(left_contours) and 1) + (len(right_contours) and 1) + (len(top_contours) and 1)) > 1
    is_bend = len(top_contours) == 0 and (len(left_contours) + len(right_contours)) == 1

    prev_frame_time = new_frame_time

    cv2.putText(frame, str(int(fps)), (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
    cv2.putText(frame, str(height) + "x" + str(width), (10, height - 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)
    cv2.drawContours(main_frame, contours, -1, (0, 255, 0), 3)

    # Display the original and masked images
    cv2.imshow("Image", frame)

    if cv2.waitKey(10) == ord("q"):
        break

capture.release()
cv2.destroyAllWindows()