#!/usr/bin/env python3
import os
import sys

def process_xacro(xacro_path, output_path):
    """Convert xacro to URDF while preserving package:// URIs"""
    
    try:
        # Import xacro as a Python module
        import xacro
        
        print(f"Processing {xacro_path}...")
        
        # Process the xacro file
        doc = xacro.process_file(xacro_path)
        
        # Write the output
        with open(output_path, 'w') as f:
            f.write(doc.toprettyxml(indent='  '))
        
        print(f"Successfully generated {output_path}")
        return True
        
    except Exception as e:
        print(f"Error processing xacro: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    xacro_file = "/home/host/onda/ros_ws/braco_description/urdf/Braço.xacro"
    urdf_file = "/home/host/onda/ros_ws/braco_description/urdf/braco.urdf"
    
    success = process_xacro(xacro_file, urdf_file)
    
    if success:
        print(f"\n✓ Generated {urdf_file}")
        print("✓ Package URIs preserved for use with roboticstoolbox")
    else:
        print("\n✗ Failed to generate URDF")
        sys.exit(1)
