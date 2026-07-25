import re
with open("/home/ace/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf", "r") as f:
    content = f.read()
    includes = re.finditer(r'<include>.*?</include>', content, re.DOTALL)
    for match in includes:
        if 'iris_with_gimbal' in match.group(0):
            print("--- DRONE INCLUDE BLOCK ---")
            print(match.group(0))
            print("---------------------------")
