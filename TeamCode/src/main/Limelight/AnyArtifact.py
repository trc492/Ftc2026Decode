import cv2
import numpy as np
import math
import time

# Limelight expects these specific functions
def runPipeline(image, llrobot):
    """
    Main pipeline function called by Limelight.

    Args:
        image: Input image from camera
        llrobot: Limelight robot data object

    Returns:
        contours: List of detected contours
        image: Processed image for display
        corners: Corner points (empty list if not applicable)
    """
    start_time = time.time()

    detector = GameBallDetector()
    balls, annotated_image = detector.detect_balls(image)

    # Calculate FPS
    processing_time = time.time() - start_time
    fps = 1.0 / processing_time if processing_time > 0 else 0

    # Draw FPS on image
    cv2.putText(annotated_image, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Convert balls to Limelight contour format
    contours = []
    for ball in balls:
        # Create a circular contour for each detected ball
        center = (ball['center_x'], ball['center_y'])
        radius = ball['radius']

        # Generate circle points
        points = []
        for i in range(16):  # 16-point circle approximation
            angle = 2 * math.pi * i / 16
            x = int(center[0] + radius * math.cos(angle))
            y = int(center[1] + radius * math.sin(angle))
            points.append([x, y])

        contour = np.array(points, dtype=np.int32)
        contours.append(contour)

    # Send numeric data to NetworkTables (purple=1, green=2)
    color_codes = {'purple': 1, 'green': 2}

    if balls:
        # Send: [num_balls, fps, x1, y1, color1, x2, y2, color2, ...]
        llpython = [len(balls), round(fps, 1)]  # First element is count, second is FPS
        for ball in balls:
            llpython.extend([
                ball['center_x'],
                ball['center_y'],
                color_codes.get(ball['color'], 0)
            ])
    else:
        llpython = [0, round(fps, 1)]  # No balls detected, but still send FPS

    return contours, annotated_image, llpython

class GameBallDetector:
    """Compact OpenCV pipeline for detecting colored game balls."""

    def __init__(self):
        self.color_ranges = {
            'purple': [120, 50, 50, 160, 255, 255],
            'green': [40, 50, 50, 80, 255, 255]
        }

        self.min_area = 200.0
        self.size_range = [20, 200]
        self.min_circle_distance = 30.0
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        self.colors = {
            'purple': (255, 0, 255),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'default': (0, 255, 0)
        }

    def detect_balls(self, image):
        """Main detection function."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_balls = []
        annotated = image.copy()

        for color_name, (h_min, s_min, v_min, h_max, s_max, v_max) in self.color_ranges.items():
            mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            balls = self._detect_circles(mask, color_name)
            detected_balls.extend(balls)
            self._annotate_balls(annotated, balls, color_name)

        return detected_balls, annotated

    def _detect_circles(self, mask, color_name):
        """Detect circles using HoughCircles."""
        blurred = cv2.GaussianBlur(mask, (5, 5), 2)

        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.0,
            minDist=self.min_circle_distance,
            param1=100, param2=30,
            minRadius=self.size_range[0]//2,
            maxRadius=self.size_range[1]//2
        )

        balls = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                area = math.pi * r * r
                if area >= self.min_area:
                    balls.append({
                        'color': color_name,
                        'center_x': int(x),
                        'center_y': int(y),
                        'radius': int(r),
                        'area': area
                    })

        return balls

    def _annotate_balls(self, image, balls, color_name):
        """Draw annotations for detected balls."""
        color = self.colors.get(color_name, self.colors['default'])

        for ball in balls:
            x, y, r = ball['center_x'], ball['center_y'], ball['radius']

            # Draw circle outline
            cv2.circle(image, (x, y), r, color, 3)

            # Draw center point
            cv2.circle(image, (x, y), 3, color, -1)

            # Draw label
            label = ball['color']
            cv2.putText(image, label, (x - r, y - r - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

# Alternative simple functions for testing
def detect_purple_balls(image):
    """Simple function to detect only purple balls."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (120, 50, 50), (160, 255, 255))

    # Clean mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter and annotate
    annotated = image.copy()
    valid_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 200:
            # Draw bounding circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            if 10 < radius < 100:  # Size filter
                cv2.circle(annotated, center, radius, (255, 0, 255), 3)
                cv2.circle(annotated, center, 3, (255, 0, 255), -1)
                cv2.putText(annotated, "purple", (center[0]-30, center[1]-radius-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                valid_contours.append(contour)

    return valid_contours, annotated

def detect_all_colors(image):
    """Detect multiple colored balls."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    annotated = image.copy()
    all_contours = []

    colors = {
        'purple': ([120, 50, 50], [160, 255, 255], (255, 0, 255)),
        'green': ([40, 50, 50], [80, 255, 255], (0, 255, 0)),
        'yellow': ([20, 50, 50], [40, 255, 255], (0, 255, 255))
    }

    for color_name, (lower, upper, bgr_color) in colors.items():
        mask = cv2.inRange(hsv, tuple(lower), tuple(upper))

        # Clean mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                if 10 < radius < 100:
                    cv2.circle(annotated, center, radius, bgr_color, 3)
                    cv2.circle(annotated, center, 3, bgr_color, -1)
                    cv2.putText(annotated, color_name, (center[0]-30, center[1]-radius-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr_color, 2)
                    all_contours.append(contour)

    return all_contours, annotated

# Test function for Limelight debugging
def simple_pipeline(image):
    """Ultra-simple pipeline for testing."""
    try:
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for purple objects
        lower_purple = np.array([120, 50, 50])
        upper_purple = np.array([160, 255, 255])
        mask = cv2.inRange(hsv, lower_purple, upper_purple)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw on original image
        result = image.copy()
        cv2.drawContours(result, contours, -1, (255, 0, 255), 2)

        return contours, result, []

    except Exception as e:
        # Return original image if error
        return [], image, [str(e)]