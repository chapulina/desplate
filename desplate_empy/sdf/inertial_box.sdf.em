@{
x = size[0]
y = size[1]
z = size[2]
}@
<inertial>
  <mass>@(mass)</mass>
  <inertia>
    <ixx>@(mass * (y*y+z*z) / 12.0)</ixx>
    <ixy>0.0</ixy>
    <ixz>0.0</ixz>
    <iyy>@(mass * (x*x+z*z) / 12.0)</iyy>
    <iyz>0.0</iyz>
    <izz>@(mass * (x*x+y*y) / 12.0)</izz>
  </inertia>
</inertial>
