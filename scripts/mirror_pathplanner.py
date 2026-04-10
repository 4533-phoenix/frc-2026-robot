import json
import os
import re

# We can dynamically get the field width if we parsed Constants.java or FieldUtil.java,
# but using the standard 2024/2025 dimension is most reliable here.
FIELD_WIDTH = 8.052

def mirror_name(name):
    if not name:
        return name
    if "Left" in name:
        return name.replace("Left", "Right")
    elif "Right" in name:
        return name.replace("Right", "Left")
    return "Right " + name

def mirror_rotation(deg):
    if deg is None:
        return None
    return -deg

def mirror_path(path_data):
    # Mirror waypoints
    for wp in path_data.get("waypoints", []):
        if wp.get("anchor"):
            wp["anchor"]["y"] = FIELD_WIDTH - wp["anchor"]["y"]
        if wp.get("prevControl"):
            wp["prevControl"]["y"] = FIELD_WIDTH - wp["prevControl"]["y"]
        if wp.get("nextControl"):
            wp["nextControl"]["y"] = FIELD_WIDTH - wp["nextControl"]["y"]
    
    # Mirror rotation targets
    for rot in path_data.get("rotationTargets", []):
        rot["rotationDegrees"] = mirror_rotation(rot["rotationDegrees"])
        
    # Mirror end state
    if "goalEndState" in path_data and path_data["goalEndState"]:
        if "rotation" in path_data["goalEndState"]:
            path_data["goalEndState"]["rotation"] = mirror_rotation(path_data["goalEndState"]["rotation"])
            
    # Mirror start state
    if "idealStartingState" in path_data and path_data["idealStartingState"]:
        if "rotation" in path_data["idealStartingState"]:
            path_data["idealStartingState"]["rotation"] = mirror_rotation(path_data["idealStartingState"]["rotation"])

    # If inside a folder with Left/Right, change that too
    if "folder" in path_data and path_data["folder"]:
        path_data["folder"] = mirror_name(path_data["folder"])

    return path_data

def mirror_command(command_data):
    if not command_data:
        return
    cmd_type = command_data.get("type")
    
    if cmd_type == "path":
        if "data" in command_data and "pathName" in command_data["data"]:
            command_data["data"]["pathName"] = mirror_name(command_data["data"]["pathName"])
            
    elif cmd_type in ["sequential", "parallel", "race", "deadline"]:
        if "data" in command_data and "commands" in command_data["data"]:
            for cmd in command_data["data"]["commands"]:
                mirror_command(cmd)

def mirror_auto(auto_data):
    if "command" in auto_data:
        mirror_command(auto_data["command"])
        
    # Optional starting pose rotation
    if "startingPose" in auto_data and auto_data["startingPose"]:
        if "position" in auto_data["startingPose"]:
            auto_data["startingPose"]["position"]["y"] = FIELD_WIDTH - auto_data["startingPose"]["position"]["y"]
        if "rotation" in auto_data["startingPose"]:
            auto_data["startingPose"]["rotation"] = mirror_rotation(auto_data["startingPose"]["rotation"])
        
    if "folder" in auto_data and auto_data["folder"]:
        auto_data["folder"] = mirror_name(auto_data["folder"])

    return auto_data

def process_file(file_path, is_auto):
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return

    name_without_ext = os.path.splitext(os.path.basename(file_path))[0]
    mirrored_name = mirror_name(name_without_ext)
    
    # Do not mirror files that are already mirrored
    if "Right" in name_without_ext and "Left" not in name_without_ext:
        return 

    if is_auto:
        mirrored_data = mirror_auto(data)
    else:
        mirrored_data = mirror_path(data)
        
    # Determine new file path
    dir_name = os.path.dirname(file_path)
    ext = os.path.splitext(file_path)[1]
    mirrored_file_path = os.path.join(dir_name, mirrored_name + ext)
    
    try:
        with open(mirrored_file_path, "w") as f:
            json.dump(mirrored_data, f, indent=2)
        print(f"Generated mirrored file: {mirrored_name}{ext}")
    except Exception as e:
        print(f"Error writing {mirrored_file_path}: {e}")

def process_settings(base_dir):
    settings_path = os.path.join(base_dir, "settings.json")
    if not os.path.exists(settings_path):
        return

    try:
        with open(settings_path, "r") as f:
            settings = json.load(f)
    except Exception as e:
        print(f"Error reading {settings_path}: {e}")
        return

    changed = False
    for folder_key in ["pathFolders", "autoFolders"]:
        if folder_key in settings:
            existing_folders = set(settings[folder_key])
            for folder in list(existing_folders):
                if folder:
                    mirrored = mirror_name(folder)
                    if mirrored != folder and mirrored not in existing_folders:
                        settings[folder_key].append(mirrored)
                        changed = True

    if changed:
        try:
            with open(settings_path, "w") as f:
                json.dump(settings, f, indent=2)
            print("Updated settings.json with mirrored folders.")
        except Exception as e:
            print(f"Error writing {settings_path}: {e}")

def main():
    # Attempt to parse field width from FieldUtil.java if applicable
    global FIELD_WIDTH
    
    base_dir = os.path.join(os.getcwd(), "src", "main", "deploy", "pathplanner")
    paths_dir = os.path.join(base_dir, "paths")
    autos_dir = os.path.join(base_dir, "autos")

    if not os.path.exists(paths_dir):
        os.makedirs(paths_dir)
    if not os.path.exists(autos_dir):
        os.makedirs(autos_dir)

    print(f"Mirroring across Y axis (Field Width = {FIELD_WIDTH}m)")

    # Mirror Paths
    for filename in os.listdir(paths_dir):
        if filename.endswith(".path"):
            process_file(os.path.join(paths_dir, filename), False)

    # Mirror Autos
    for filename in os.listdir(autos_dir):
        if filename.endswith(".auto"):
            process_file(os.path.join(autos_dir, filename), True)

    # Mirror Folders in settings.json
    process_settings(base_dir)

if __name__ == "__main__":
    main()
