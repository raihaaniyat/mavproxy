import cv2
import logging
import os
from datetime import datetime
from ultralytics import YOLO

# ==============================================================================
# MODULE: Vision & Detection
# DESCRIPTION: Reads a video feed (mocking the VTOL camera), runs YOLOv8 to 
# detect humans, and extracts the center pixel coordinates for targeting.
# ==============================================================================

def setup_logger(module_name):
    """Reusing our robust logger from Phase 1."""
    if not os.path.exists('system_logs'):
        os.makedirs('system_logs')
    log_filename = f"system_logs/vtol_{module_name}_{datetime.now().strftime('%Y-%m-%d')}.log"
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    file_handler = logging.FileHandler(log_filename)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - [%(levelname)s] - <Line: %(lineno)d> - %(message)s'))
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    return logger

log = setup_logger("VISION_MODULE")

def run_vision_loop(video_source):
    """
    Main loop for processing the camera feed.
    video_source: Path to a local video file, or 0 for a webcam.
    """
    log.info("Initializing YOLOv8 model... (This might download the model on first run)")
    # Using the 'nano' model for speed. You can upgrade to 'yolov8m.pt' or 'yolov8l.pt' later for better accuracy.
    model = YOLO("yolov8n.pt") 
    
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        log.error(f"Failed to open video source: {video_source}")
        return

    log.info("Video feed established. Beginning human detection loop.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.info("End of video stream reached.")
                break

            # Run YOLO inference on the current frame
            # classes=[0] strictly filters for 'person' in the COCO dataset
            # conf=0.5 ensures we only get confident detections to reduce false positives
            results = model.predict(source=frame, classes=[0], conf=0.5, verbose=False)

            human_count = 0
            
            # Parse the results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    human_count += 1
                    
                    # Extract bounding box coordinates (x_min, y_min, x_max, y_max)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Calculate the center pixel of the detected human
                    # This center pixel (center_x, center_y) is crucial for our future GPS math
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # Log the detection with its pixel coordinates
                    log.debug(f"Human {human_count} detected at pixel coords: X:{center_x}, Y:{center_y}")

                    # Minimal GUI: Draw a dot at the center and a bounding box for visual debugging
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1) # Red dot at center
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)    # Green bounding box
                    cv2.putText(frame, f"Human {human_count}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if human_count > 0:
                log.info(f"Frame processed: {human_count} human(s) detected.")

            # Show the minimal GUI window
            # We can disable this later when running purely headless on the VTOL companion computer
            cv2.imshow("VTOL Vision Debug", frame)

            # Press 'q' in the OpenCV window to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                log.info("Operator terminated vision loop via GUI.")
                break

    except KeyboardInterrupt:
        log.info("Operator stopped vision script via CLI (Ctrl+C).")
    except Exception as e:
        log.error(f"Vision loop crashed: {e}", exc_info=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        log.info("Camera released and vision system shut down cleanly.")

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
if __name__ == "__main__":
    # To test this, download a short drone video of people walking and put it in the same folder.
    # Replace "sample_drone_video.mp4" with your actual file name.
    # If you want to test with your laptop webcam just to see it work, change it to: video_file = 0
    video_file = "sample_drone_video.mp4" 
    
    # Create a dummy file just so OpenCV doesn't crash instantly if you forget to add a video
    if not os.path.exists(video_file) and video_file != 0:
        log.warning(f"File '{video_file}' not found. Please place a test video in the directory.")
    else:
        run_vision_loop(video_file)
