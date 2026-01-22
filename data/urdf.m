% This script parses the URDF file to obtain link masses, center of mass
% locations, link dimensions, and inertia tensors. The extracted data is
% formatted for direct use in dynamic simulations of a floating-base
% robotic arm operating in microgravity.

% Author: <Preksha Krishnan>

<?xml version="1.0" encoding="utf-8"?>
<robot name="dofbot">
    <link name="base_link">
        <inertial>
            <origin xyz="0 0 0.06" rpy="0 0 0"/>
            <mass value="0.4302"/>
            <inertia ixx="0.002669" ixy="0" ixz="0" iyy="0.000762" iyz="0" izz="0.002940"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/base_link.STL"/>
            </geometry>
            <material name="">
                <color rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/base_link.STL"/>
            </geometry>
        </collision>
    </link>
    <link name="link1">
        <inertial>
            <origin xyz="0 0 0.02745" rpy="0 0 0"/>
            <mass value="0.1129"/>
            <inertia ixx="0.000057" ixy="0" ixz="0" iyy="0.000057" iyz="0" izz="0.000057"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link1.STL"/>
            </geometry>
            <material name="">
                <color rgba="0 0.7 0 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link1.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint1" type="revolute">
        <origin xyz="0 0 0.064" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.5708" upper="1.5708" effort="100" velocity="1"/>
    </joint>
    <link name="link2">
        <inertial>
            <origin xyz="0 0 0.0238" rpy="0 0 0"/>
            <mass value="0.1458"/>
            <inertia ixx="0.000149" ixy="0" ixz="0" iyy="0.000167" iyz="0" izz="0.000038"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link2.STL"/>
            </geometry>
            <material name="">
                <color rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link2.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint2" type="revolute">
        <origin xyz="0 0 0.0435" rpy="0 1.5708 0"/>
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.5708" upper="1.5708" effort="100" velocity="1"/>
    </joint>
    <link name="link3">
        <inertial>
            <origin xyz="0 0 0.0238" rpy="0 0 0"/>
            <mass value="0.1459"/>
            <inertia ixx="0.000149" ixy="0" ixz="0" iyy="0.000167" iyz="0" izz="0.000038"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link3.STL"/>
            </geometry>
            <material name="">
                <color rgba="0 0.7 0 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link3.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint3" type="revolute">
        <origin xyz="-0.08285 0 0" rpy="0 0 0"/>
        <parent link="link2"/>
        <child link="link3"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.5708" upper="1.5708" effort="100" velocity="1"/>
    </joint>
    <link name="link4">
        <inertial>
            <origin xyz="0 0 0.01755" rpy="0 0 0"/>
            <mass value="0.2209"/>
            <inertia ixx="0.000323" ixy="0" ixz="0" iyy="0.000173" iyz="0" izz="0.000196"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link4.STL"/>
            </geometry>
            <material name="">
                <color rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link4.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint4" type="revolute">
        <origin xyz="-0.08285 0 0" rpy="0 0 0"/>
        <parent link="link3"/>
        <child link="link4"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.5708" upper="1.5708" effort="100" velocity="1"/>
    </joint>
    <link name="link5">
        <inertial>
            <origin xyz="0 0 0.03025" rpy="0 0 0"/>
            <mass value="0.2003"/>
            <inertia ixx="0.000257" ixy="0" ixz="0" iyy="0.000266" iyz="0" izz="0.000113"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link5.STL"/>
            </geometry>
            <material name="">
                <color rgba="1 1 1 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <mesh filename="meshes/link5.STL"/>
            </geometry>
        </collision>
    </link>
    <joint name="joint5" type="revolute">
        <origin xyz="-0.07385 -0.00215 0" rpy="0 -1.5708 0"/>
        <parent link="link4"/>
        <child link="link5"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.5708" upper="3.1416" effort="100" velocity="1"/>
    </joint>
</robot>
