from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
model = YOLO('yolov8s.pt' )
import cv2
import numpy as np
import torch
# Convert HEX to RGB
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    lv = len(hex_color)
    return tuple(int(hex_color[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

# Convert RGB to HSV
def rgb_to_hsv(r, g, b):
    color = np.uint8([[[b, g, r]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    return hsv_color[0][0]

# Load image
def load_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Could not load image {image_path}")
    return image

# Calculate brightness
def calculate_brightness(mask):
    return np.sum(mask) / np.count_nonzero(mask) if np.count_nonzero(mask) else 0

# Define the color ranges in HSV space
green_on_hsv = rgb_to_hsv(*hex_to_rgb("#78F569"))
green_off_hsv = rgb_to_hsv(*hex_to_rgb("#20712F"))
red_on_hsv = rgb_to_hsv(*hex_to_rgb("#FB6B51"))
red_off_hsv = rgb_to_hsv(*hex_to_rgb("#79414E"))

# Function to create a mask for a given color
def create_mask(hsv_image, color_hsv):
    lower_bound = np.array([color_hsv[0] - 10, max(color_hsv[1] - 40, 100), max(color_hsv[2] - 40, 100)])
    upper_bound = np.array([color_hsv[0] + 10, min(color_hsv[1] + 40, 255), min(color_hsv[2] + 40, 255)])
    return cv2.inRange(hsv_image, lower_bound, upper_bound)

def disI(x1,y1,x2,y2,image):
    x1, y1, x2, y2 = x1,y1,x2,y2
    # print(x1)
    box_width = x2 - x1
    box_height = y2 - y1
    frame_width = image.shape[1]
    frame_height = image.shape[0]
    frame_area = frame_width*frame_height
    box_area= box_width * box_height
    distance = (box_area / frame_area) * 100
    return round(distance,3)
# Process each image
def process_images(yoloimage):
    if yoloimage is None:
        return
    hsv_image = cv2.cvtColor(yoloimage, cv2.COLOR_BGR2HSV)
    results = {}
    masks = {
        'green_on': create_mask(hsv_image, green_on_hsv),
        'green_off': create_mask(hsv_image, green_off_hsv),
        'red_on': create_mask(hsv_image, red_on_hsv),
        'red_off': create_mask(hsv_image, red_off_hsv)
    }
    brightness = {color: calculate_brightness(mask) for color, mask in masks.items()}
    light_status = 'green' if brightness['green_on'] > brightness['green_off'] else 'red'
    results= light_status        
    return results

def mainlogic(gflag,dis):
        if gflag == "green":
            return 'green'
        # elif gflag == "red" and (dis >=0.50 and dis <= 0.75):
        elif gflag == "red" and (dis >=0.50):
            return "stop"

        elif gflag == "stop" and (dis >=0.50):
            return "stop"

        else:
            return 'pass'


def mov_logic(image):
    results = model(image,classes=[9,11],conf=0.7,verbose=False)  # return a list of Results objects
    # results = model(image)  # return a list of Results objects
    # Process results list
    for result in results:
        boxes = result.boxes  # Boxes object for bounding box outputs
        if not torch.equal(torch.tensor([]),boxes.cls):    
            if boxes.cls[0].item()==9.0: # traffic ligth
                x1, y1, x2, y2 = map(int, boxes.xyxy[0])  # Get bounding box coordinates
                cropped_image = image[y1:y2, x1:x2]  #
                dis=disI(x1,y1,x2,y2,image)
                # print(dis([x1,y1,x2,y2],image))
                Getflag =process_images(cropped_image)
                return mainlogic(Getflag,dis)
                 

            elif boxes.cls[0].item()==11.0: # stop sign
                x1, y1, x2, y2 = map(int, boxes.xyxy[0])  # Get bounding box coordinates
                cropped_image = image[y1:y2, x1:x2]
                # print(dis([x1,y1,x2,y2],image))
                dis=disI(x1,y1,x2,y2,image)
                Getflag ="stop"
                return mainlogic(Getflag,dis)


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

