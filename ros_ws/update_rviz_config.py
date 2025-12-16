import sys

file_path = '/home/host/onda/ros_ws/src/braco_description/launch/urdf.rviz'

replacements = {
    'Class: rviz/Displays': 'Class: rviz_common/Displays',
    'Class: rviz/Selection': 'Class: rviz_common/Selection',
    'Class: rviz/Tool Properties': 'Class: rviz_common/Tool Properties',
    'Class: rviz/Views': 'Class: rviz_common/Views',
    'Class: rviz/Time': 'Class: rviz_common/Time',
    'Class: rviz/Grid': 'Class: rviz_default_plugins/Grid',
    'Class: rviz/RobotModel': 'Class: rviz_default_plugins/RobotModel',
    'Class: rviz/TF': 'Class: rviz_default_plugins/TF',
    'Class: rviz/Orbit': 'Class: rviz_default_plugins/Orbit',
    'Class: rviz/Interact': 'Class: rviz_default_plugins/Interact',
    'Class: rviz/MoveCamera': 'Class: rviz_default_plugins/MoveCamera',
    'Class: rviz/Select': 'Class: rviz_default_plugins/Select',
    'Class: rviz/FocusCamera': 'Class: rviz_default_plugins/FocusCamera',
    'Class: rviz/Measure': 'Class: rviz_default_plugins/Measure',
    'Class: rviz/SetInitialPose': 'Class: rviz_default_plugins/SetInitialPose',
    'Class: rviz/SetGoal': 'Class: rviz_default_plugins/SetGoal',
    'Class: rviz/PublishPoint': 'Class: rviz_default_plugins/PublishPoint',
}

with open(file_path, 'r') as f:
    content = f.read()

for old, new in replacements.items():
    content = content.replace(old, new)

with open(file_path, 'w') as f:
    f.write(content)

print("Updated urdf.rviz")
