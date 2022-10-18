@{
# Generates one of the actuated cylindrical wheels

y_offset = wheel_offset[1] if side == "left" else -wheel_offset[1]

}@
<joint name="@(side)_wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="@(side)_wheel"/>
  <origin
    xyz="@(wheel_offset[0]) @(y_offset) @(wheel_offset[2])"
    rpy="-@(math.pi*0.5) 0 0" />
  <axis xyz="0 0 1"/>
</joint>
<link name="@(side)_wheel">
  <visual>
    <geometry>
      <cylinder radius="@(wheel_radius)" length="@(wheel_length)"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="@(wheel_radius)" length="@(wheel_length)"/>
    </geometry>
  </collision>
@{
empy.include(template_path("inertial_cylinder.urdf.em"), {
  "mass": wheel_mass,
  "radius": wheel_radius,
  "length": wheel_length,
})
}@
</link>
