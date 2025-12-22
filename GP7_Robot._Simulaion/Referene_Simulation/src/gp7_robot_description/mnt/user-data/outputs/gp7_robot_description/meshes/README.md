# Mesh Files Directory

This directory should contain the STL mesh files for the GP7 Robot visualization and collision detection.

## Required Files

You need to copy the following STL files from your SolidWorks URDF export:

- `base_link.STL`
- `Link_1.STL`
- `Link_2.STL`
- `Link_3.STL`
- `Link_4.STL`
- `Link_5.STL`
- `Link_6.STL`

## How to Add Mesh Files

From your original SolidWorks export location (e.g., `/path/to/GP7 Robot.SLDASM/meshes/`):

```bash
cp /path/to/original/export/meshes/*.STL /path/to/gp7_robot_description/meshes/
```

Or if you're in the package directory:

```bash
cp /path/to/original/export/meshes/*.STL ./meshes/
```

## Verification

After copying the files, verify they're present:

```bash
ls -lh meshes/
```

You should see 7 STL files with reasonable file sizes (typically a few KB to a few MB each).

## Rebuild After Adding Meshes

After adding mesh files, rebuild the package:

```bash
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description
source install/setup.bash
```

## Troubleshooting

If meshes don't load in RViz:

1. Check file permissions:
   ```bash
   chmod 644 meshes/*.STL
   ```

2. Verify package is installed:
   ```bash
   ros2 pkg prefix gp7_robot_description
   ```

3. Check mesh paths in URDF:
   ```bash
   grep "meshes" urdf/gp7_robot.urdf
   ```

All paths should start with `package://gp7_robot_description/meshes/`
