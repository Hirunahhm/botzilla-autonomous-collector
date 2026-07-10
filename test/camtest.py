import cv2
import numpy as np
from PIL import Image
import freenect

def get_depth_and_rgb():
    depth, timestamp = freenect.sync_get_depth()
    rgb, timestamp = freenect.sync_get_video()
    return depth, rgb
# Function to detect the color
def detect_color(frame):
    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # print(hsv)

    # Define the lower and upper boundaries of the color you want to detect (in HSV)
    lower_bound_b = np.array([100, 100, 100])  # Lower bound of the color (here, it's for Blue color)
    upper_bound_b = np.array([140, 255, 255])  # Upper bound of the color (here, it's for Blue color)

    lower_bound_g = np.array([0,100,100])  # Lower bound of the color (here, it's for green color)
    upper_bound_g = np.array([10,255,255])  # Upper bound of the color (here, it's for green color)

    # Create a mask for the blue color range
    global mask1
    mask1 = cv2.inRange(hsv, lower_bound_b, upper_bound_b)
    # Create a mask for the green color range
    global mask2
    mask2 = cv2.inRange(hsv, lower_bound_g, upper_bound_g)

    # Combine the green object and background
    global result
    result = cv2.add(mask1, mask2)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=result)

    return res

# Function to determine the dominant color
def determine_color():
    blue_pixels = cv2.countNonZero(mask1)
    green_pixels = cv2.countNonZero(mask2)

    if blue_pixels > green_pixels:
        return ["Blue",blue_pixels]
    elif green_pixels > blue_pixels:
        return ["Red",green_pixels]
    else:
        return "No dominant color"

blue_pix = list()
red_pix = list()
blue_depth = list()
red_depth = list()
# Open a video capture object (you can replace '0' with your video file name if you want to process a video)
cap = cv2.VideoCapture(1)

while True:
    depth, rgb = get_depth_and_rgb()
    # Convert depth data to a grayscale 
    depth = depth.astype("uint8") >> 2  # Right shift to make 10-bit data 8-bit
    depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
    # Read a frame from the video capture
    # ret, frame = cap.read()
    frame =rgb
    # Call the detect_color function to get the detected color in the frame
    detected_color = detect_color(frame)
    # Apply Canny edge detection (example)
    edges = cv2.Canny(detected_color, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mask_box_b = Image.fromarray(result)
    bbox_b = mask_box_b.getbbox()
    if bbox_b is not None:
        x1, y1, x2, y2 = bbox_b
        print(abs(x1-x2),abs(y1-y2))
        if abs(x1-x2)>200 and abs(y1-y2)>200:
            # frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 265, 0), 2)
            print(determine_color())
            if determine_color()[0] == "Blue":
                blue_pix.append(determine_color()[1])
                blue_depth.append(depth)
                frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (255,0,0), 2)
            elif determine_color()[0] == "Red":
                red_pix.append(determine_color()[1])
                red_depth.append(depth)
                frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0,255), 2)

    # Display the original frame and the detected color
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Detected Color', detected_color)
    cv2.imshow('Depth',depth)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#print("r",red_pix,red_depth)
print("b",blue_depth)
# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()