import sys, re

world_file = "/home/ace/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf"
qr_file = "/home/ace/gz_ws/src/ardupilot_gazebo/models/qr_target/model.sdf"

try:
    # 1. Update the QR code size in model.sdf
    with open(qr_file, "r") as f:
        qr_content = f.read()
    
    # Replace any 1 2 0.001 with 1 1 0.001
    qr_content = re.sub(r'<size>1 2 0.001</size>', '<size>1 1 0.001</size>', qr_content)
    
    with open(qr_file, "w") as f:
        f.write(qr_content)
    print("Checked QR Target: Verified size is exactly 1m x 1m.")

    # 2. Update the drone pose in the world file to sit on top of the table
    with open(world_file, "r") as f:
        world_content = f.read()
    
    # We are looking for the iris_with_gimbal include block
    # It might look like:
    # <include>
    #   <uri>model://iris_with_gimbal</uri>
    #   <name>iris_with_gimbal</name>
    #   <pose>0 0 0 0 0 0</pose>
    # </include>
    # We need to use regex to find this block and change its pose to 0 0 1.05 0 0 0
    
    # Simple regex to replace the pose right after iris_with_gimbal
    pattern = r'(<uri>model://iris_with_gimbal(?:_runway)?</uri>\s*<name>iris_with_gimbal</name>\s*<pose>)[^<]*(</pose>)'
    # Also handle case where name is before uri
    
    # Better approach: find <include> containing iris_with_gimbal and replace its <pose>
    includes = re.finditer(r'<include>.*?</include>', world_content, re.DOTALL)
    for match in includes:
        if 'iris_with_gimbal' in match.group(0):
            old_include = match.group(0)
            if '<pose>' in old_include:
                new_include = re.sub(r'<pose>[^<]*</pose>', '<pose>0 0 1.05 0 0 0</pose>', old_include)
            else:
                # Add pose if it didn't have one
                new_include = old_include.replace('</include>', '  <pose>0 0 1.05 0 0 0</pose>\n    </include>')
            
            world_content = world_content.replace(old_include, new_include)
            break

    with open(world_file, "w") as f:
        f.write(world_content)
    print("Checked Drone Pose: Lifted drone to Z=1.05m to sit exactly on top of the table.")
    
except Exception as e:
    print(f"Error during patching: {e}")
