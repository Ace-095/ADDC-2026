import sys

filename = "/home/ace/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf"
try:
    with open(filename, "r") as f:
        content = f.read()
        
    if "drone_table" in content:
        print("Table is already in the file!")
        sys.exit(0)
        
    table_xml = """
    <model name="drone_table">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 1.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 1.5 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
"""
    # Insert before the closing </world> tag
    content = content.replace("</world>", table_xml + "\n  </world>")
    
    with open(filename, "w") as f:
        f.write(content)
        
    print(f"Successfully added the 1m table to {filename}")
except Exception as e:
    print(f"Error: {e}")
