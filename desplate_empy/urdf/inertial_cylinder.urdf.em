@{
xy = mass * (3 * radius * radius + length * length) / 12.0
z = mass * radius * radius * 0.5
}@
<inertial>
  <mass value="@(mass)" />
  <inertia
    ixx="@(xy)"
    ixy="0.0"
    ixz="0.0"
    iyy="@(xy)"
    iyz="0.0"
    izz="@(z)" />
</inertial>
