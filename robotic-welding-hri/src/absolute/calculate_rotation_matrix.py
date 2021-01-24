import math
import numpy as np

# Converts URx's rotation vector into a rotation matrix
# 
# I did not derive this nor do I fully understand the maths behind this :0
# I took it from: https://dof.robotiq.com/discussion/1648/around-which-axes-are-the-rotation-vector-angles-defined
def convert_tool_pose_to_transformation_matrix(tool_pose):  
  r = tool_pose[3:]
  rx = r[0]
  ry = r[1]
  rz = r[2]

  theta = math.sqrt( (rx ** 2) + (ry ** 2) + (rz ** 2) )

  ux = rx / theta
  uy = ry / theta
  uz = rz / theta

  c = math.cos(theta)
  s = math.sin(theta)
  C = 1 - c

  # base_to_tcp = np.array([0, -600, -135])
  base_to_tcp = tool_pose[:3]

  T = np.array([
    [(ux * ux * C) + c       ,   (ux * uy * C) - (uz * s),    (ux * uz * C) + (uy * s), base_to_tcp[0]],
    [(uy * ux * C) + (uz * s),   (uy * uy * C) + c       ,    (uy * uz * C) - (ux * s), base_to_tcp[1]],
    [(uz * ux * C) - (uy * s),   (uz * uy * C) + (ux * s),    (uz * uz * C) + c       , base_to_tcp[2]],
    [0,0,0,1]
  ])
  return T

# Calculates hand position in absolute coordinates
def calculate_hand_position(transformation_matrix, relative_palm_postion):
  # Formats raw hand coordinates
  hand_coordinates_raw =  relative_palm_postion
  hand_coordinates_raw =  [50, 0, 0]
  hand_coordinates_raw.append(1)
  hand_coordinates = np.array(hand_coordinates_raw) * [1, -1, 1, 1]

  # Gets abolsolute matrix by transformation matrix multiplication
  absolute_position =  transformation_matrix.dot(hand_coordinates)
  return np.round(absolute_position[:3],3)


def calculate_required_robot_position(absolute_hand_position, y_offset=0):
  required_robot_position = absolute_hand_position + [0, 130, 0]
  # required_robot_position = absolute_hand_position + y_offset
  return required_robot_position

def main():
  tool_pose = [50, -600, -135, 0, 3.14, 0]
  T = convert_tool_pose_to_transformation_matrix(tool_pose)
  relative_palm_postion = [0, 103, 0]

  absolute_hand_position = calculate_hand_position(T, relative_palm_postion)
  print(absolute_hand_position)

  required_robot_position = calculate_required_robot_position(absolute_hand_position)
  print(required_robot_position)

main()
  
