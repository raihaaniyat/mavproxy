import os
import subprocess
import shutil

def prepare_and_run_odm(source_images_folder, project_folder):
    """
    Prepares the ODM workspace and runs the photogrammetry engine.
    """
    # 1. Build the strict ODM folder structure
    images_dest = os.path.join(project_folder, "images")
    
    if not os.path.exists(images_dest):
        os.makedirs(images_dest)
        print(f"Created ODM project workspace at: {project_folder}")

    # 2. Gather and copy the images
    print(f"Scanning {source_images_folder} for images...")
    
    # Filter for standard image formats
    valid_extensions = ('.jpg', '.jpeg', '.png', '.tif', '.tiff')
    image_files = [f for f in os.listdir(source_images_folder) if f.lower().endswith(valid_extensions)]
    
    if not image_files:
        print(f"Error: No valid images found in {source_images_folder}.")
        return

    print(f"Found {len(image_files)} images. Copying to workspace...")
    for img in image_files:
        src_path = os.path.join(source_images_folder, img)
        dest_path = os.path.join(images_dest, img)
        
        # Only copy if it doesn't already exist (saves time if you run the script twice)
        if not os.path.exists(dest_path):
            shutil.copy2(src_path, dest_path)
            
    print("Images successfully prepared.")

    # 3. Trigger Docker and ODM
    print("\n--- Starting OpenDroneMap ---")
    print("Warning: This process will heavily utilize your CPU.\n")

    abs_project_path = os.path.abspath(project_folder)
    
    command = [
        "docker", "run", "--rm",
        "-v", f"{abs_project_path}:/code",
        "opendronemap/odm",
        "--project-path", "/code",
        "--fast-orthophoto" # Remove this flag if you want 3D models alongside the 2D map
    ]

    try:
        subprocess.run(command, check=True)
        
        print("\n--- ODM Processing Complete! ---")
        ortho_path = os.path.join(abs_project_path, "odm_orthophoto", "odm_orthophoto.tif")
        print(f"Your final map should be located at: {ortho_path}")
        
    except subprocess.CalledProcessError as e:
        print(f"\nError: ODM process failed with exit code {e.returncode}.")
    except FileNotFoundError:
        print("\nError: Docker is not installed or not found in your system PATH. Please ensure Docker Desktop is running.")

# ==========================================
# Execution Block
# ==========================================
if __name__ == "__main__":
    # 1. Point this to your existing folder containing the raw photos
    MY_RAW_PHOTOS = "./my_drone_photos" 
    
    # 2. Define where you want ODM to build its structure and save the final map
    MY_ODM_WORKSPACE = "./odm_mapping_project"
    
    prepare_and_run_odm(MY_RAW_PHOTOS, MY_ODM_WORKSPACE)