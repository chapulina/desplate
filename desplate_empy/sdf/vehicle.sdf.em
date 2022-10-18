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
  return os.path.join(pkg, "sdf", file_path)

def material(color_name):
    """Prints a material with a given color"""
    if color_name == "grey":
        color = "0.8 0.8 0.8 1"
    elif color_name == "black":
        color = "0 0 0 1"
    print("""
        <material>
          <diffuse>%s</diffuse>
          <specular>%s</specular>
          <ambient>%s</ambient>
        </material>
    """ % (color, color, color))

}@
<sdf version="1.7">
<model name="vehicle">
  <link name="chassis">
    <visual name="visual">
      <geometry>
        <box>
          <size>
            @(chassis_size[0])
            @(chassis_size[1])
            @(chassis_size[2])
          </size>
        </box>
      </geometry>
      @(material("grey"))
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>
            @(chassis_size[0])
            @(chassis_size[1])
            @(chassis_size[2])
          </size>
        </box>
      </geometry>
    </collision>
@{
empy.include(template_path("inertial_box.sdf.em"), {
  "mass": chassis_mass,
  "size": chassis_size
})
}@
  </link>

@{empy.include(template_path("wheel.sdf.em"), {"side": "left"})}@
@{empy.include(template_path("wheel.sdf.em"), {"side": "right"})}@

    <!-- caster -->
    <!-- TODO(chapulina) Use ball - currently doesn't work with joint_state_publisher -->
    <joint name="caster_wheel_joint" type="fixed">
      <parent>chassis</parent>
      <child>caster_wheel</child>
    </joint>
    <link name="caster_wheel">
      <pose>
        @(caster_offset[0])
        @(caster_offset[1])
        @(caster_offset[2])
        0 0 0
      </pose>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>@(caster_radius)</radius>
          </sphere>
        </geometry>
        @(material("black"))
      </visual>
      <collision name="collision">
        <geometry>
            <sphere>
              <radius>@(caster_radius)</radius>
            </sphere>
        </geometry>
      </collision>
@{
empy.include(template_path("inertial_sphere.sdf.em"), {
  "mass": caster_mass,
  "radius": caster_radius
})
}@
  </link>

</model>
</sdf>
