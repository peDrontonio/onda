#!/usr/bin/env python3
import os
import re

def process_xacro_manual(xacro_path, output_path):
    """Convert xacro to URDF by manually resolving includes and substitutions"""
    
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(xacro_path)))
    print(f"Package directory: {package_dir}")
    
    # Read main xacro file
    with open(xacro_path, 'r') as f:
        content = f.read()
    
    # Replace $(find braco_description) with actual path BUT keep it as package://
    # We DON'T want to replace package:// - that's the whole point!
    # We only need to resolve the $(find ...) in xacro:include directives
    
    # Find and process includes
    include_pattern = r'<xacro:include filename="\$\(find braco_description\)/(.*?)" />'
    includes = re.findall(include_pattern, content)
    
    for inc_rel_path in includes:
        inc_path = os.path.join(package_dir, inc_rel_path)
        print(f"Processing include: {inc_path}")
        
        if not os.path.exists(inc_path):
            print(f"Warning: Include file not found: {inc_path}")
            continue
        
        with open(inc_path, 'r') as f:
            inc_content = f.read()
        
        # Remove XML declaration from include if present
        inc_content = re.sub(r'<\?xml.*?\?>\s*', '', inc_content)
        
        # Remove robot wrapper from include if present
        # (keep the content inside <robot> tags)
        robot_match = re.search(r'<robot.*?>(.*)</robot>', inc_content, re.DOTALL)
        if robot_match:
            inc_content = robot_match.group(1)
        
        # Replace the include directive with the actual content
        content = content.replace(
            f'<xacro:include filename="$(find braco_description)/{inc_rel_path}" />',
            inc_content
        )
    
    # Remove xacro namespace declaration
    content = content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')
    
    # Process xacro:property tags (simple properties only)
    prop_pattern = r'<xacro:property name="(.*?)" value="(.*?)" />'
    properties = {}
    for match in re.finditer(prop_pattern, content):
        prop_name = match.group(1)
        prop_value = match.group(2)
        properties[prop_name] = prop_value
        print(f"Property: {prop_name} = {prop_value}")
    
    # Remove property declarations
    content = re.sub(prop_pattern, '', content)
    
    # Replace property references ${property_name}
    for prop_name, prop_value in properties.items():
        content = content.replace(f'${{{prop_name}}}', prop_value)
    
    # Clean up extra whitespace
    content = re.sub(r'\n\s*\n', '\n', content)
    
    # Write output
    with open(output_path, 'w') as f:
        f.write(content)
    
    print(f"Successfully generated {output_path}")
    print("✓ Package URIs (package://braco_description) are preserved!")
    return True

if __name__ == "__main__":
    xacro_file = "/home/host/onda/ros_ws/braco_description/urdf/Braço.xacro"
    urdf_file = "/home/host/onda/ros_ws/braco_description/urdf/braco.urdf"
    
    try:
        success = process_xacro_manual(xacro_file, urdf_file)
        if success:
            print(f"\n✓ Generated {urdf_file}")
            print("✓ Ready to use with roboticstoolbox and Swift!")
    except Exception as e:
        print(f"\n✗ Failed: {e}")
        import traceback
        traceback.print_exc()
