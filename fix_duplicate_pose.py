import re

world_file = "/home/ace/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf"

with open(world_file, "r") as f:
    content = f.read()

# We need to find the specific iris_with_gimbal include block and clean it up
def replacer(match):
    block = match.group(0)
    if 'iris_with_gimbal' in block:
        # Reconstruct the block cleanly
        return """    <include>
      <uri>model://iris_with_gimbal</uri>
      <name>iris_with_gimbal</name>
      <pose degrees="true">0 0 1.5 0 0 90</pose>
    </include>"""
    return block

content = re.sub(r'<include>.*?</include>', replacer, content, flags=re.DOTALL)

with open(world_file, "w") as f:
    f.write(content)
    
print("Successfully fixed duplicate pose tags and set drone to Z=1.5m, facing down the runway (90 deg).")
