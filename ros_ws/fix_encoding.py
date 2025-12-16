import os
import codecs

def convert_to_utf8(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.xacro', '.xml', '.trans', '.gazebo', '.launch')):
                filepath = os.path.join(root, file)
                try:
                    # Try reading as utf-8 first
                    with codecs.open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                except UnicodeDecodeError:
                    print(f"Converting {filepath} from latin-1 to utf-8")
                    # If failed, try latin-1 (common for 'ç')
                    with codecs.open(filepath, 'r', encoding='latin-1') as f:
                        content = f.read()
                    
                    # Write back as utf-8
                    with codecs.open(filepath, 'w', encoding='utf-8') as f:
                        f.write(content)

if __name__ == "__main__":
    convert_to_utf8("/home/host/ros_ws/braco_description")
