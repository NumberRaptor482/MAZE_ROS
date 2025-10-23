from math import sqrt

name = input("Enter name: ")

DIMENSION_RATIO = 2#0.4826

def wall_to_surface(sx, sy, ex, ey, link_name)->str:
    height = 2#0.3302 # 13 inches

    sx *= DIMENSION_RATIO # 19 inch squares
    sy *= DIMENSION_RATIO
    ex *= DIMENSION_RATIO
    ey *= DIMENSION_RATIO

    length = sqrt((ey - sy) ** 2 + (ex - sx) ** 2)

    mid_x = (sx + ex) / 2
    mid_y = (sy + ey) / 2

    angle = 0 if sy == ey else 1.5707963267948966

    color = '1 0 0 1' if angle == 0 else '0 1 0 1'

    link = f"""    <link name="{link_name}">
      <pose>{mid_x} {mid_y} {height / 2} 0 0 {angle}</pose>
      <collision name="{link_name}_collision">
        <geometry>
          <box>
            <size>{length} 0.1 2.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="{link_name}_visual">
        <geometry>
          <box>
            <size>{length} 0.1 2.5</size>
          </box>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>"""

    return link

data = open(name + '.map').readlines()

width, height = map(float, data[0].split(','))

width *= DIMENSION_RATIO
height *= DIMENSION_RATIO

output = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
        <link name="floor">
            <pose>{width / 2} {height / 2} 0 0 0 0</pose>
            <collision name="floor_collision">
                <geometry>
                <plane>
                    <normal>0 0 1</normal>
                    <size>{width} {height}</size>
                </plane>
                </geometry>
            </collision>
            <visual name="floor_visual">
                <geometry>
                <plane>
                    <normal>0 0 1</normal>
                    <size>{width} {height}</size>
                </plane>
                </geometry>
            </visual>
        </link>"""

for ln, line in enumerate(data[1:]):
    sx, ex, sy, ey = map(float, line.split(','))
    output += wall_to_surface(sx, sy, ex, ey, 'wall_' + str(ln))

output += """
  </model>
</sdf>"""

open(name + '.sdf', 'w').write(output)