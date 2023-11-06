"""
This file is part of vulcan_location.

Copyright (C) 2023 Vulcan YJX <vulcanyjx@163.com>
For more information see <https://github.com/Vulcan-YJX/vulcan_location>

vulcan_location is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

vulcan_location is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
"""

image_height = 720
image_width = 1280
bf = 32.3252131124
D_left = [-0.04087948, 0.03006104, 0, 0, 0.0]
K_left = [639.2645, 0, 635.7686, 0, 639.5627, 355.1494, 0, 0, 1]
R_left = [0.9999935662, -0.0033233921, -0.0013500365, 0.0033244884, 0.9999941453, 0.0008106159, 0.0013473346, -0.0008150989, 0.9999987602]
T_left = [150, 0, 0]
D_right = [-0.04023259, 0.02970617, 0, 0, 0.0]
K_right = [639.7958, 0, 641.171, 0, 639.9821, 354.8851, 0, 0, 1]
R_right = [0.9999943337, -0.0022228281, 0.0025281572, 0.0022248824, 0.9999971969, -0.0008100474, -0.0025263495, 0.0008156677, 0.9999964761]
T_right = [150.6438364153, 0, 0]


with open('camera_info.yaml', 'w') as f:
    f.write('%YAML:1.0\n\n')
    f.write(f'image_height: {image_height}\n')
    f.write(f'image_width: {image_width}\n')
    f.write(f'bf: {bf}\n\n')
    f.write('LEFT.D: !!opencv-matrix\n   rows: 1\n   cols: 4\n   dt: d\n   data: {}\n'.format(D_left))
    f.write('LEFT.K: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: {}\n'.format(K_left))
    f.write('LEFT.R: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: {}\n'.format(R_left))
    f.write('LEFT.T: !!opencv-matrix\n   rows: 1\n   cols: 3\n   dt: d\n   data: {}\n\n'.format(T_left))
    f.write('RIGHT.D: !!opencv-matrix\n   rows: 1\n   cols: 4\n   dt: d\n   data: {}\n'.format(D_right))
    f.write('RIGHT.K: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: {}\n'.format(K_right))
    f.write('RIGHT.R: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: {}\n'.format(R_right))
    f.write('RIGHT.T: !!opencv-matrix\n   rows: 1\n   cols: 3\n   dt: d\n   data: {}\n'.format(T_right))
