# import xml.etree.ElementTree as ET

# def parse_text_file(input_file):
#     with open(input_file, 'r') as file:
#         lines = file.readlines()
    
#     # Ignore the first line (header) and parse the joint positions
#     data = [list(map(float, line.strip().split(','))) for line in lines[1:] if line.strip()]
#     return data

# def generate_kuka_xml(data, output_file):
#     root = ET.Element("joint_trajectory")
#     ET.SubElement(root, "frame", id="")
    
#     # Define joint names (KUKA arm has 7 joints, adjust accordingly)
#     joint_names = [f"kuka_joint_{i+1}" for i in range(len(data[0]))]
#     for joint in joint_names:
#         ET.SubElement(root, "joint", name=joint)
    
#     ET.SubElement(root, "points", number=str(len(data)))
    
#     for positions in data:
#         point_elem = ET.SubElement(root, "point")
#         for pos in positions:
#             ET.SubElement(point_elem, "position", value=str(pos))
        
#         # Time stamp (optional, assuming uniform time intervals)
#         ET.SubElement(point_elem, "time_from_start", value="0")
    
#     tree = ET.ElementTree(root)
#     tree.write(output_file, encoding="utf-8", xml_declaration=True)
#     print(f"XML file saved as {output_file}")

# if __name__ == "__main__":
#     input_file = "Kuka_trajectories.txt"  # Change to your actual file name
#     output_file = "kuka_trajectory.xml"
    
#     data = parse_text_file(input_file)
#     generate_kuka_xml(data, output_file)


import xml.etree.ElementTree as ET
import xml.dom.minidom

def convert_text_to_kuka_xml(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    # Extract joint values from the input file
    joint_values = []
    for line in lines[1:]:  # Skip the first line (header)
        values = line.strip().split(',')
        if len(values) > 1:
            joint_values.append([float(v) for v in values if v])  # Convert to floats and ignore empty strings

    # Create XML structure
    root = ET.Element("joint_trajectory")
    ET.SubElement(root, "frame", id="")

    # Define joints for a 7-DOF KUKA arm
    joints = [ET.SubElement(root, "joint", name=f"kuka_joint_{i+1}") for i in range(7)]
    
    # Number of points
    ET.SubElement(root, "points", number=str(len(joint_values)))

    # Populate points
    for values in joint_values:
        point = ET.SubElement(root, "point")
        for value in values:
            ET.SubElement(point, "position", value=str(value))
        ET.SubElement(point, "time_from_start", value="0")  # Modify time values as needed

    # Pretty-print XML
    xml_str = ET.tostring(root, encoding="utf-8")
    parsed_xml = xml.dom.minidom.parseString(xml_str)
    formatted_xml = parsed_xml.toprettyxml(indent="    ")

    # Save to file
    with open(output_file, "w") as f:
        f.write(formatted_xml)

# Usage example:
convert_text_to_kuka_xml("kuka_trajectories.txt", "kuka_trajectory.xml")