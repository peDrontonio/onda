import os

file_path = '/home/host/onda/ros_ws/src/braco_description/urdf/Braço.xacro'

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

new_content = content.replace('package://braco_description/meshes/', 'file://$(find braco_description)/meshes/')

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_content)

print("Replaced package:// with file://$(find ...)")
