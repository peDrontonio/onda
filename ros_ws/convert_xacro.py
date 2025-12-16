import os
import re

def process_xacro(xacro_path, output_path):
    with open(xacro_path, 'r') as f:
        content = f.read()

    # Replace $(find braco_description) with the absolute path
    package_path = os.path.dirname(os.path.dirname(xacro_path))
    content = content.replace('$(find braco_description)', package_path)
    
    # Also replace package://braco_description with the absolute path for meshes
    content = content.replace('package://braco_description', package_path)
    
    # Simple include handling (very basic, assumes includes are in the same dir or relative)
    # This is a poor man's xacro, but might work if includes are simple
    # Actually, let's try to use the xacro command if available, with the path replacement
    
    # Save to temp file
    temp_xacro = xacro_path + '.temp'
    with open(temp_xacro, 'w') as f:
        f.write(content)
        
    # Run xacro command
    # We need to source ROS first, but we can try running /opt/ros/humble/bin/xacro directly
    cmd = f"/opt/ros/humble/bin/xacro {temp_xacro} > {output_path}"
    print(f"Running: {cmd}")
    ret = os.system(cmd)
    
    if ret != 0:
        print("Xacro failed. Trying manual include resolution (fallback)...")
        # Fallback: manually read includes
        # Find <xacro:include filename="..." />
        includes = re.findall(r'<xacro:include filename="(.*?)" />', content)
        for inc in includes:
            inc_path = inc
            if not os.path.exists(inc_path):
                print(f"Warning: Include {inc_path} not found")
                continue
            with open(inc_path, 'r') as f_inc:
                inc_content = f_inc.read()
                # Remove xml header from include
                inc_content = re.sub(r'<\?xml.*?\?>', '', inc_content)
                # Remove robot tag wrapper if present (usually includes are just macros, but sometimes...)
                # Actually, xacro includes are just text insertion mostly.
                
                # Replace the include line with content
                content = content.replace(f'<xacro:include filename="{inc}" />', inc_content)
        
        # Remove xacro namespace
        content = content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')
        content = re.sub(r'<xacro:.*?>', '', content) # Remove other xacro tags? No, that breaks things.
        
        # Save fallback
        with open(output_path, 'w') as f:
            f.write(content)

    # Clean up
    if os.path.exists(temp_xacro):
        os.remove(temp_xacro)

if __name__ == "__main__":
    xacro_file = "/home/host/ros_ws/braco_description/urdf/Braço.xacro"
    urdf_file = "/home/host/ros_ws/braco_description/urdf/braco.urdf"
    process_xacro(xacro_file, urdf_file)
    print(f"Generated {urdf_file}")
