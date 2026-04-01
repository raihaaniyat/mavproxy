import cv2
import os
import subprocess

def extract_frames(video_path, project_folder, capture_rate=1):
    """Extracts frames and saves them in the folder structure ODM expects."""
    # ODM expects images to be inside an 'images' subfolder within the project directory
    images_folder = os.path.join(project_folder, "images")
    
    if not os.path.exists(images_folder):
        os.makedirs(images_folder)

    vidcap = cv2.VideoCapture(video_path)
    fps = vidcap.get(cv2.CAP_PROP_FPS)
    frame_skip = int(fps * capture_rate)
    
    success, image = vidcap.read()
    count = 0
    saved_count = 0

    print(f"Starting frame extraction to {images_folder}...")

    while success:
        if count % frame_skip == 0:
            frame_name = os.path.join(images_folder, f"frame_{saved_count:04d}.jpg")
            cv2.imwrite(frame_name, image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
            saved_count += 1
            
        success, image = vidcap.read()
        count += 1

    print(f"Extraction complete. {saved_count} images ready for ODM.")
    return saved_count

def run_odm(project_folder):
    """Triggers the ODM Docker container to process the images."""
    print("\n--- Starting OpenDroneMap ---")
    print("Warning: This process will heavily utilize your CPU and may take a long time.\n")

    # Docker requires absolute paths to mount volumes correctly
    abs_project_path = os.path.abspath(project_folder)
    
    # Construct the Docker command
    # --fast-orthophoto skips building a 3D model to generate the 2D map much faster.
    # Remove it if you also want 3D meshes (.obj) and point clouds (.las).
    command = [
        "docker", "run", "--rm",
        "-v", f"{abs_project_path}:/code",
        "opendronemap/odm",
        "--project-path", "/code",
        "--fast-orthophoto" 
    ]

    try:
        # subprocess.run will execute the command and stream the ODM logs to your terminal
        process = subprocess.run(command, check=True)
        
        print("\n--- ODM Processing Complete! ---")
        ortho_path = os.path.join(abs_project_path, "odm_orthophoto", "odm_orthophoto.tif")
        print(f"Your final map should be located at: {ortho_path}")
        
    except subprocess.CalledProcessError as e:
        print(f"\nError: ODM process failed with exit code {e.returncode}.")
    except FileNotFoundError:
        print("\nError: Docker is not installed or not found in your system PATH. Please install Docker Desktop and ensure it is running.")

# ==========================================
# Execution Block
# ==========================================
if __name__ == "__main__":
    # 1. Define your inputs
    # Use a short video clip for your first test!
    MY_VIDEO = "drone_flight.mp4" 
    MY_PROJECT_DIR = "./my_mapping_project"
    
    # 2. Extract the frames
    # Setting capture_rate to 0.5 takes a frame every half-second (more overlap)
    frames_extracted = extract_frames(MY_VIDEO, MY_PROJECT_DIR, capture_rate=1)
    
    # 3. Run ODM if frames were successfully extracted
    if frames_extracted > 0:
        run_odm(MY_PROJECT_DIR)
    else:
        print("No frames were extracted. Check your video file path.")