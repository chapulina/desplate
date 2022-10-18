@{
# Generates one of the actuated cylindrical wheels

y_offset = wheel_offset[1] if side == "left" else -wheel_offset[1]

}@
<!-- TODO(chapulina) Use revolute - currently doesn't work with joint_state_publisher -->
<joint name="@(side)_wheel_joint" type="fixed">
  <parent>chassis</parent>
  <child>@(side)_wheel</child>
  <axis>
    <xyz>0 0 1</xyz>
  </axis>
</joint>
<link name="@(side)_wheel">
  <pose>
    @(wheel_offset[0])
    @(y_offset)
    @(wheel_offset[2])
    -@(math.pi*0.5)
    0
    0
  </pose>
  <visual name="visual">
    <geometry>
      <cylinder>
        <radius>@(wheel_radius)</radius>
        <length>@(wheel_length)</length>
      </cylinder>
    </geometry>
    @(material("black"))
  </visual>
  <collision name="collision">
    <geometry>
      <cylinder>
        <radius>@(wheel_radius)</radius>
        <length>@(wheel_length)</length>
      </cylinder>
    </geometry>
  </collision>
@{
empy.include(template_path("inertial_cylinder.sdf.em"), {
  "mass": wheel_mass,
  "radius": wheel_radius,
  "length": wheel_length,
})
}@
</link>
