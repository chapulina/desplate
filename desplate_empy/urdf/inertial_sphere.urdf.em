@{
inertia = (2 * mass * radius * radius) / 5.0
}@
<inertial>
  <mass value="@(mass)" />
  <inertia
    ixx="@(inertia)"
    ixy="0.0"
    ixz="0.0"
    iyy="@(inertia)"
    iyz="0.0"
    izz="@(inertia)" />
</inertial>
