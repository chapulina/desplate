@{
x = size[0]
y = size[1]
z = size[2]
}@
<inertial>
  <mass value="@(mass)" />
  <inertia
    ixx="@(mass * (y*y+z*z) / 12.0)"
    ixy="0.0"
    ixz="0.0"
    iyy="@(mass * (x*x+z*z) / 12.0)"
    iyz="0.0"
    izz="@(mass * (x*x+y*y) / 12.0)" />
</inertial>
