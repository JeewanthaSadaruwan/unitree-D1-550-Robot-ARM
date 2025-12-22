#!/usr/bin/env python3
"""
URDF Validation Script for GP7 Robot
Checks the URDF file for common issues
"""

import sys
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def validate_urdf(urdf_path):
    """Validate URDF file"""
    
    print("=" * 60)
    print("GP7 Robot URDF Validation")
    print("=" * 60)
    print(f"\nValidating: {urdf_path}\n")
    
    if not os.path.exists(urdf_path):
        print(f"❌ ERROR: URDF file not found: {urdf_path}")
        return False
    
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"❌ ERROR: Failed to parse URDF: {e}")
        return False
    
    print("✓ URDF file is valid XML")
    
    # Check robot name
    robot_name = root.get('name')
    print(f"✓ Robot name: {robot_name}")
    
    # Count links and joints
    links = root.findall('link')
    joints = root.findall('joint')
    
    print(f"✓ Found {len(links)} links")
    print(f"✓ Found {len(joints)} joints")
    
    # Expected configuration
    expected_links = ['base_link', 'Link_1', 'Link_2', 'Link_3', 'Link_4', 'Link_5', 'Link_6']
    expected_joints = ['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
    
    # Validate links
    print("\nLink Validation:")
    link_names = [link.get('name') for link in links]
    
    for expected in expected_links:
        if expected in link_names:
            print(f"  ✓ {expected}")
        else:
            print(f"  ❌ Missing: {expected}")
    
    # Check for extra links
    extra_links = set(link_names) - set(expected_links)
    if extra_links:
        print(f"  ⚠ Extra links found: {extra_links}")
    
    # Validate joints
    print("\nJoint Validation:")
    joint_names = [joint.get('name') for joint in joints]
    
    for expected in expected_joints:
        if expected in joint_names:
            joint = [j for j in joints if j.get('name') == expected][0]
            joint_type = joint.get('type')
            
            # Check joint limits
            limit = joint.find('limit')
            if limit is not None:
                lower = limit.get('lower')
                upper = limit.get('upper')
                effort = limit.get('effort')
                velocity = limit.get('velocity')
                
                # Check for zero effort/velocity (common issue)
                if effort == '0' or velocity == '0':
                    print(f"  ⚠ {expected} ({joint_type}): effort={effort}, velocity={velocity} (should not be 0)")
                else:
                    print(f"  ✓ {expected} ({joint_type}): [{lower}, {upper}] effort={effort} vel={velocity}")
            else:
                print(f"  ⚠ {expected} ({joint_type}): No limits defined")
        else:
            print(f"  ❌ Missing: {expected}")
    
    # Validate mesh files
    print("\nMesh File Validation:")
    package_dir = Path(urdf_path).parent.parent
    meshes_dir = package_dir / 'meshes'
    
    mesh_elements = root.findall('.//mesh')
    mesh_files = set()
    
    for mesh in mesh_elements:
        filename = mesh.get('filename')
        if filename:
            # Extract just the filename from package:// URI
            mesh_file = filename.split('/')[-1]
            mesh_files.add(mesh_file)
    
    print(f"  URDF references {len(mesh_files)} unique mesh files")
    
    if meshes_dir.exists():
        actual_meshes = list(meshes_dir.glob('*.STL'))
        print(f"  Found {len(actual_meshes)} STL files in meshes directory")
        
        for mesh_file in sorted(mesh_files):
            mesh_path = meshes_dir / mesh_file
            if mesh_path.exists():
                size = mesh_path.stat().st_size
                print(f"    ✓ {mesh_file} ({size:,} bytes)")
            else:
                print(f"    ❌ {mesh_file} (not found)")
    else:
        print(f"  ⚠ Meshes directory not found: {meshes_dir}")
    
    # Validate inertial properties
    print("\nInertial Properties:")
    links_without_inertia = []
    
    for link in links:
        link_name = link.get('name')
        inertial = link.find('inertial')
        
        if inertial is None:
            links_without_inertia.append(link_name)
        else:
            mass_elem = inertial.find('mass')
            if mass_elem is not None:
                mass = float(mass_elem.get('value', 0))
                if mass == 0:
                    print(f"  ⚠ {link_name}: mass is 0")
    
    if links_without_inertia:
        print(f"  ⚠ Links without inertial properties: {links_without_inertia}")
    else:
        print(f"  ✓ All links have inertial properties")
    
    # Validate parent-child relationships
    print("\nKinematic Chain:")
    parent_child = {}
    for joint in joints:
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            parent_link = parent.get('link')
            child_link = child.get('link')
            parent_child[child_link] = parent_link
    
    # Build chain from base_link
    def print_chain(link, indent=0):
        print("  " + "  " * indent + f"└─ {link}")
        children = [child for child, parent in parent_child.items() if parent == link]
        for child in children:
            print_chain(child, indent + 1)
    
    print_chain('base_link')
    
    print("\n" + "=" * 60)
    print("Validation Complete")
    print("=" * 60)
    
    return True


if __name__ == '__main__':
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]
    else:
        # Try to find URDF in standard location
        script_dir = Path(__file__).parent
        urdf_path = script_dir / 'urdf' / 'gp7_robot.urdf'
    
    success = validate_urdf(str(urdf_path))
    sys.exit(0 if success else 1)
