
import json
from pathlib import Path


# ==================================================
# CONFIGURATION
# ==================================================

SCRIPT_DIR = Path(__file__).resolve().parent

JSON_FILE = SCRIPT_DIR / "experiments copy.json"
IMAGE_FOLDER = SCRIPT_DIR / "experiment_camera_frames"

# False = simulation
# True = actual deletion
DELETE_FILES = True


# ==================================================
# EXTRACT IMAGE PATHS
# ==================================================

def extract_image_paths(data):

    referenced_images = set()

    def extract_paths(obj):

        """Recursively search for every image_path."""

        if isinstance(obj, dict):

            for key, value in obj.items():

                if key == "image_path" and isinstance(value, str):

                    filename = Path(value.replace("\\", "/")).name

                    if filename:
                        referenced_images.add(filename)

                else:
                    extract_paths(value)

        elif isinstance(obj, list):

            for item in obj:
                extract_paths(item)

    def recursive_search(obj):

        """Find every ROS_output in the complete JSON structure."""

        if isinstance(obj, dict):

            for key, value in obj.items():

                if key == "ROS_output":

                    # Convert ROS_output string into dictionary
                    if isinstance(value, str):

                        try:
                            ros_data = json.loads(value)

                            # Search all image_path fields
                            extract_paths(ros_data)

                        except json.JSONDecodeError as error:

                            print(
                                f"WARNING: Invalid ROS_output: {error}"
                            )

                    elif isinstance(value, (dict, list)):

                        extract_paths(value)

                else:

                    recursive_search(value)

        elif isinstance(obj, list):

            for item in obj:
                recursive_search(item)

    recursive_search(data)

    return referenced_images


# ==================================================
# CLEAN IMAGE FOLDER
# ==================================================

def clean_image_folder():

    # Load JSON
    print(f"\nReading JSON: {JSON_FILE.name}")

    with open(JSON_FILE, "r", encoding="utf-8") as file:
        data = json.load(file)

    # Extract all referenced images
    referenced_images = extract_image_paths(data)

    # Check folder
    if not IMAGE_FOLDER.exists():

        print(f"ERROR: Folder not found: {IMAGE_FOLDER}")
        return

    # Get all files recursively
    existing_images = [
        p for p in IMAGE_FOLDER.rglob("*")
        if p.is_file()
    ]

    initial_count = len(existing_images)

    # Identify files not referenced in JSON
    files_to_delete = [
        image for image in existing_images
        if image.name not in referenced_images
    ]

    # ==================================================
    # INITIAL REPORT
    # ==================================================

    print("\n" + "=" * 55)
    print("IMAGE CLEANUP REPORT")
    print("=" * 55)

    print(f"Initial images in folder: {initial_count}")
    print(f"Images referenced in JSON: {len(referenced_images)}")
    print(f"Images to remove: {len(files_to_delete)}")

    print("\nReferenced images:")

    for name in sorted(referenced_images):
        print(f"  {name}")

    # ==================================================
    # DELETE FILES
    # ==================================================

    removed_count = 0

    if DELETE_FILES:

        print("\nRemoving unused images...")

        for image in files_to_delete:

            try:
                image.unlink()
                removed_count += 1

                print(f"Removed: {image.name}")

            except OSError as error:

                print(f"ERROR deleting {image}: {error}")

        print("\nDeletion completed.")

    else:

        print("\nDRY RUN: no files were deleted.")
        print("Set DELETE_FILES = True to enable deletion.")

    # ==================================================
    # FINAL REPORT
    # ==================================================

    final_count = sum(
        1 for p in IMAGE_FOLDER.rglob("*")
        if p.is_file()
    )

    print("\n" + "=" * 55)
    print("FINAL STATISTICS")
    print("=" * 55)

    print(f"Initial images: {initial_count}")
    print(f"Removed images: {removed_count}")
    print(f"Final images:   {final_count}")

    print("=" * 55)


# ==================================================
# MAIN
# ==================================================

if __name__ == "__main__":
    clean_image_folder()
