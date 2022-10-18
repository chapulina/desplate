<?xml version="1.0"?>
@{

import math
import os
from ament_index_python.packages import get_package_share_directory

# Inputs
chassis_size = [1.0, 0.5, 0.15]
chassis_mass = 1.0

wheel_radius = 0.15
wheel_length = 0.05
wheel_mass = 0.5

caster_radius = 0.1
caster_mass = 0.0415553

# Calculated properties
wheel_offset = [
  chassis_size[0] * 0.3,
  (chassis_size[1] + wheel_length) * 0.5,
  -wheel_radius * 0.5]

caster_offset = [
  -chassis_size[0] * 0.3,
  0,
  -(chassis_size[2] + caster_radius) * 0.5]

pkg = os.path.join(get_package_share_directory('desplate_empy'))

def template_path(file_path):
  return os.path.join(pkg, "urdf", file_path)

}@
<robot name="vehicle">
  <material name="grey">
    <color rgba="0.8 0.8 0.8 1" />
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <link name="chassis">
    <visual>
      <geometry>
        <box size="@(chassis_size[0]) @(chassis_size[1]) @(chassis_size[2])"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="@(chassis_size[0]) @(chassis_size[1]) @(chassis_size[2])"/>
      </geometry>
    </collision>
@{
empy.include(template_path("inertial_box.urdf.em"), {
  "mass": chassis_mass,
  "size": chassis_size
})
}@
  </link>

@{empy.include(template_path("wheel.urdf.em"), {"side": "left"})}@
@{empy.include(template_path("wheel.urdf.em"), {"side": "right"})}@

  <!-- caster -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="chassis"/>
    <child link="caster_wheel"/>
    <origin xyz="@(caster_offset[0]) @(caster_offset[1]) @(caster_offset[2])"/>
  </joint>
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="@(caster_radius)"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="@(caster_radius)"/>
      </geometry>
    </collision>
@{
empy.include(template_path("inertial_sphere.urdf.em"), {
  "mass": caster_mass,
  "radius": caster_radius
})
}@
  </link>

</robot>
