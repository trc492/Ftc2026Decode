import cv2
import numpy as np

# Constants for the camera
HORIZONTAL_FOV = 54.5  # degrees
VERTICAL_FOV = 42.0  # degrees
KNOWN_WIDTH = 0.127  # meters (adjust this to the known width of your target)
FOCAL_LENGTH = 620  # pixels (this is an estimate, you may need to calibrate)

def calculate_distance_and_angles(x, y, w, h, img_width, img_height):
    # Calculate distance
    distance = (KNOWN_WIDTH * FOCAL_LENGTH) / w * 39.37

    # Calculate angles
    horizontal_angle = (x - img_width/2) * (HORIZONTAL_FOV / img_width)
    vertical_angle = (img_height/2 - y) * (VERTICAL_FOV / img_height)

    return distance, horizontal_angle, vertical_angle

def drawDecorations(image, text):
    cv2.putText(image, text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    # cv2.line(image, (0, 410), (639, 410), (0, 255, 0), 2)

def runPipeline(image, llrobot):
    # Convert BGR to YCrCb
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)

    # Define range for green color in YCrCb
    lower_green = np.array([100, 120, 30])
    upper_green = np.array([250, 200, 80])
    # For green object, lower hsv = (52, 187, 79)
    # For green object, lower bgr = (21, 79, 36)
    # For green object, higher hsv = (50, 105, 199)
    # For green object, higher bgr = (119, 199, 145)
    # I think these are right, the link for the photos is here: https://tinyurl.com/decodeobj
    
    # Create mask for green color
    mask = cv2.inRange(ycrcb, lower_green, upper_green)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)

    # Find contours
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables
    largest_contour = np.array([[]])
    distance = 0
    horizontal_angle = 0
    vertical_angle = 0
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Calculate distance and angles
        distance, horizontal_angle, vertical_angle = calculate_distance_and_angles(
            x + w/2, y + h/2, w, h, image.shape[1], image.shape[0])
        
        # Draw rectangle around the largest contour
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Draw center point
        center = (int(x + w/2), int(y + h/2))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
        # Prepare data to send back to the robot
        llpython = [1, distance, horizontal_angle, vertical_angle, x, y, w, h]

    # Draw text on image
    text = f"HAngle:{horizontal_angle:.2f}deg, VAngle:{vertical_angle:.2f}deg, Dist:{distance:.2f}in"
    drawDecorations(image, text)

    return largest_contour, image, llpython