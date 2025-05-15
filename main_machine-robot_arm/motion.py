import serial
import time
import math
import cv2
import numpy as np

# Colour code
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
RESET="\033[0m"

# Constant Variables
arm_length = 150.0 # length of arm
cam_length = 165.0 # length of camera

# Offset for frame for kinematics
base_frame = 0
arm1_frame = 0
arm2_frame = 30
cam_frame = 65

# Offset for actual angle for scaling
base_offset = -5
arm1_offset = 15
arm2_offset = 15
cam_offset = 2

# Gripper variable
gripper_state = 0 # 0 = open, 1 = close
ready_gripper = 100.0 # Servo angle for opened gripper
close_gripper = 155.0 # Servo angle for closed gripper

# State of robot (checking or scanning/pick/place)
check_state = 1 # 0 = check, 1 = ready
thetha_check = 60.0 # angle relative to the table surface for checking
thetha_cam = 90.0 # angle relative to the table surface for finding green box, pick and place

# Additional configuration
normal_speed = 40 # Speed of motor
pick_dist = 25 # Distance between each rectangle grid at pick area
grip2cam_dis = 30 # Distance between the cam centre and gripper centre
# Pick position
ready_z = 175 # Elevation for ready/check grid
pick_z1 = 100 # Minimum elevation for picking at grid 1
pick_z2 = 103 # Minimum elevation for picking at grid 2
pick_z3 = 105 # Minimum elevation for picking at grid 3
place_z1 = 70 # Minimum elevation for placing the assembly
place_z2 = 130 # Minimum elevation for stacking the assembly
max_extension = 200 # Maximum an arm can extend for pick & placement
grid_dist = 20 # Distance between assembly
min_place_dist = 30 # Tangential distance for placing assmebly
max_place_dist = 50 # Radial distance for placing assembly

# Initial position x-y-z
robot_pos = {
  "check_assembly_position" : [0, 100, ready_z, gripper_state, check_state], # Standby position to pick/check grid
  "scan_position" : [0, 10, 200, gripper_state, check_state], # Position to scan
  "check_green_box" : [180.0, 56.50125, 25.49625, 182.0, ready_gripper] # scan_robot[0] = base servo depending on the time it detects green box
}

ready_pos = {
  "ready_position1" : [0, 100 + grid_dist , ready_z], # Standby position to pick/check grid
  "ready_position2" : [0, 100 + 2 * grid_dist, ready_z], # Standby position to pick/check grid
  "ready_position3" : [0, 100 + 3 * grid_dist, ready_z], # Standby position to pick/check grid
}

pick_pos = {
  "pick_position1" : [0, 100 + grid_dist, pick_z1], # Standby position to pick/check grid
  "pick_position2" : [0, 100 + 2 * grid_dist, pick_z2], # Standby position to pick/check grid
  "pick_position3" : [0, 100 + 3 * grid_dist, pick_z3], # Standby position to pick/check grid
}

box_pos = {
  "box_position1" : [0, 0, 0], # 1st position to place (xyz): gripper state needs to be adjust
  "box_position2" : [0, 0, 0], # 2nd position to place (xyz): gripper state needs to be adjust
  "box_position3" : [0, 0, 0], # 3rd position to place (xyz): gripper state needs to be adjust
}

place_pos = {
  "place_position1" : [0, 0, 0], # 1st position to place (xyz): gripper state needs to be adjust
  "place_position2" : [0, 0, 0], # 2nd position to place (xyz): gripper state needs to be adjust
  "place_position3" : [0, 0, 0], # 3rd position to place (xyz): gripper state needs to be adjust
}

box_place_pos = {
  "box_position1" : [0, 0, 0, 0, 0], # 1st position to place (xyz): gripper state needs to be adjust
  "box_position2" : [0, 0, 0, 0, 0], # 2nd position to place (xyz): gripper state needs to be adjust
  "box_position3" : [0, 0, 0, 0, 0], # 3rd position to place (xyz): gripper state needs to be adjust
  "place_position1" : [0, 0, 0, 0, 0], # 1st position to place (xyz): gripper state needs to be adjust
  "place_position2" : [0, 0, 0, 0, 0], # 2nd position to place (xyz): gripper state needs to be adjust
  "place_position3" : [0, 0, 0, 0, 0], # 3rd position to place (xyz): gripper state needs to be adjust
}

# Replaceable servo angle array
int_robot = [0, 0, 0, 0, 0] # Initial position
pos_robot = [0, 0, 0, 0, 0] # Target position
extra_robot = [0, 0, 0, 0, 0] # Extra position (only used in checking green box)

def on_off_motor(channel, on, ser):
  """Turns a motor on or off by sending a command via serial."""
  print(f"{GREEN}Successfully turn on motor {RESET}")
  first_byte = 0b11000000 | channel
  ser.write(bytes([first_byte, on]))

def set_ch_pos_spd(channel, position, velocity, ser):
    """Sends position and speed commands to the motor controller."""
    first_byte = 0b11100000 | channel
    if channel == 2:
      position = scale(position, 0, 180, 0, 180 + base_offset)

    if channel == 3:
      position = scale(position, 0, 180, 0, 180 + arm1_offset)

    if channel == 4:
      position = scale(position, 0, 180, 0, 180 + arm2_offset)

    if channel == 5:
      position = scale(position, 0, 180, 0, 180 + cam_offset)
    
    position = int ((position / 180.0) * 8000.0)
    high_byte = (position >> 6) & 0b01111111
    low_byte = position & 0b00111111
    ser.write(bytes([first_byte, high_byte, low_byte, velocity]))

def check(ser):
  checked = []
  for channel in range (2, 7):
    command = 0b10100000 | channel # Mode to request angle from servo controller
    ser.write(bytes([command]))
    first = ser.read(1) # read high byte
    second = ser.read(1) # read low byte
    first_byte = first[0]
    second_byte = second[0]
    position = (first_byte << 6) | (second_byte & 0x3F)
    position = (position /8000) * 180
    checked.append(position)
  checked = unscale_all(checked)
  return checked

def unscale(angle, lower_limit, upper_limit, lower_bound = 0, upper_bound = 180):
  # To convert to the scaled or read angle from servo controller back to actual angle
  init = ((((angle - lower_limit) / (upper_limit - lower_limit)) * (upper_bound - lower_bound)) + lower_bound)
  return init

def unscale_all(position): # Unscale all angle
  unscaled = [0, 0, 0, 0, 0]
  unscaled[0] = scale(position[0], 0, 180, 0, 180 + base_offset)
  unscaled[1] = unscale(position[1], 0, 180, 0, 180 + arm1_offset)
  unscaled[2] = unscale(position[2], 0, 180, 0, 180 + arm2_offset)
  unscaled[3] = unscale(position[3], 0, 180, 0, 180 + cam_offset)
  unscaled[4] = position[4]
  return unscaled

def scale(init, lower_limit, upper_limit, lower_bound = 0, upper_bound = 180):
  # To scale between actual and servo controller angle
  # Example: Servo is controlled to 180 degrees but actual angle is 185 degrees
  if init > upper_bound:
    print(f"{YELLOW}Unreachable angle{RESET} of {init}, can only reach until {upper_bound}...")
    print("Reaching minimum actual allowable angle")
    init = upper_bound
  angle = ((((init - lower_bound) / (upper_bound - lower_bound)) * (upper_limit - lower_limit)) + lower_limit)
  return angle

def movePos(x, y, z): # Inverse kinematics algorithm
  # coordinate of x, y, Z
  # grip = 0 (open), <other> (close)
  # state = 1 (check for green box, pick & place) OR perpendicular to ground surface, <other> (scan for green box) OR tilted from ground surface
  try:
    # Angle for base servo motor
    if ((x == 0.0) and  (y > 0.0)):
      pos_base = 180.0
    elif ((x > 0.0) and  (y > 0.0)):
      base_angle = math.atan2(y, x) * (180 / 3.142) # base angle
      pos_base = 90.0 + base_angle
    elif ((x > 0.0) and  (y == 0.0)):
      pos_base = 90.00
    elif ((x > 0.0) and (y < 0.0)):
      base_angle = math.atan2(-y, x) * (180 / 3.142) # base angle
      pos_base = 90.0 - base_angle
    elif ((x == 0.0) and (y < 0.0)):
      pos_base = 0.0
    else:
      pos_base = 180.0

    # Secondly find the angle of arm1 servo in 2D plane
    l = math.sqrt((x)*(x) + (y)*(y))
    h = math.sqrt(l*l + (z)*(z))
    phi = math.acos(l / h) * (180.0 / 3.142); # angle relative to l-axis
    if (z < 0.0):
      phi = -(phi)

    thetha = math.acos((h / 2.0) / arm_length) * (180.0 / 3.142)
    a1 = phi + thetha
    pos_arm1 = 180 - a1
    a2 = phi - thetha
    pos_arm2 = (90 - a1) + a2 + arm2_frame

    if check_state == 0: # Check for green box
      pos_state = thetha_check
    else: # Ready
      pos_state = thetha_cam
    
    pos_cam = pos_state + a2 + cam_frame

    if gripper_state == 0: # Open gripper
      pos_grip = ready_gripper
    else: # Close gripper
      pos_grip = close_gripper

    # Verify the angle of servo rotation is within moveable angle
    servo_angle = [int(pos_base), int(pos_arm1), int(pos_arm2), int(pos_cam), int(pos_grip)]
    for angle in servo_angle:
      if angle > 180 or angle < 0: # Moveable angle (0 <= angle <= 180)
        return False
      
    # Return all servo angle
    return servo_angle
  
  except (ValueError, ZeroDivisionError) as e:
    # Handle math error by returning a special error value or logging
    print(f"{RED}Math error{RESET} in kinematics calculation: {BLUE}{e}{RESET}")
    return False # or some default/fallback list, e.g., [0, 0, 0, 0, 0]

def robot_arm_setup(state, serial):
  global check_state, int_robot, pos_robot
  if state == 1: # Get the servo angle for all ready and pick position
    robot_pos["check_assembly_position"] = movePos(robot_pos["check_assembly_position"][0], robot_pos["check_assembly_position"][1], robot_pos["check_assembly_position"][2])
    
    if robot_pos["check_assembly_position"] == False:
      return False
    
    for position in ready_pos.values():
      servo_angle = movePos(position[0], position[1], position[2])
      if servo_angle == False:
        return False

    for position in pick_pos.values():
      servo_angle = movePos(position[0], position[1], position[2])
      if servo_angle == False:
        return False

    # Turn on motor
    on_off_motor(0, 1, serial)
    # Move to ready position
    int_robot = robot_pos["check_assembly_position"]
    rotateToAngle(int_robot, serial)
    pos_robot = int_robot
    return True
  
  else:
    check_state = 0
    robot_pos["scan_position"] = movePos(robot_pos["scan_position"][0], robot_pos["scan_position"][1], robot_pos["scan_position"][2])
    pos_robot = robot_pos["scan_position"]
    move_robot(serial)

def move_robot(serial):
  global int_robot, pos_robot
  speed = speed_equalizer()
  rotateToAngle(pos_robot, serial, speed)
  int_robot = pos_robot

def forward_kinematics(servo_angle):
  global check_state, gripper_state
  # Find the x, y, z, grip, state
  pos_base = servo_angle[0]
  pos_arm1 = servo_angle[1]
  pos_arm2 = servo_angle[2]
  pos_cam = servo_angle[3]
  pos_grip = servo_angle[4]

  if check_state == 0:
    pos_state = thetha_check
  else:
    pos_state = thetha_cam

  a2 = pos_cam - pos_state - cam_frame
  a1 = 180 - pos_arm1
  # a1 = 90 + a2 + arm2_frame - pos_arm2
  thetha = (a1 - a2) / 2
  phi = a1 - thetha
  h = math.cos(thetha * (3.142 / 180.0)) * arm_length * 2.0
  l = math.cos(phi * (3.142 / 180.0)) * h
  z = math.sqrt((h)*(h) - (l)*(l))

  if pos_base > 90.0:
    base_angle = 180.0 - pos_base
    y = math.cos((base_angle * (3.142 / 180.0))) * l
    x = math.sin((base_angle * (3.142 / 180.0))) * l
  else:
    base_angle = pos_base
    y = - (math.cos((base_angle * (3.142 / 180.0))) * l)
    x = math.sin((base_angle * (3.142 / 180.0))) * l

  coordinate = [x, y, z]

  return coordinate

def scan_box(state, serial, orientation = 1): # orientation = 1 ()

  global int_robot, pos_robot, extra_robot, check_state, max_extension

  servo_angle = check(serial) # Check servo angle for stopping

  if state == 1: # Rotate back to 0 degree

    if (servo_angle[0] == 180):
      pos_robot[0] = 0
      rotateToAngle(pos_robot, serial, speed = [15, 20, 20, 20, 20])
      return False

    elif (servo_angle[0] == 0):
      pos_robot[0] = 180
      rotateToAngle(pos_robot, serial, speed = [35, 20, 20, 20, 20])
      return True

  elif state == 2: # Position to scan for green box

    rotateToAngle(servo_angle, serial) # Stop motor

    extra_robot = servo_angle # Remember current servo angle for swapping if not succeed or math error
    robot_pos["check_green_box"][0] = servo_angle[0] # Get to re-scan position
    rotateToAngle(robot_pos["check_green_box"], serial)
    int_robot = robot_pos["check_green_box"] 

    check_state = 1 # gripper facing surface (normal)
    coordinate = forward_kinematics(robot_pos["check_green_box"])
    x = coordinate[0]
    y = coordinate[1]
    z = math.sqrt((x)*(x) + (y)*(y))
    steps = max_extension / z
    coordinate[0] = steps * coordinate[0]
    coordinate[1] = steps * coordinate[1]
    pos_robot = movePos(coordinate[0], coordinate[1], coordinate[2])

  elif state == 3: # Move towards the green box
    move_robot(serial)

  elif state == 4: # Stop when green box is found
    pos_robot = servo_angle
    rotateToAngle(pos_robot, serial)

  elif state == 5: # Robot arm reaches its maximum extension
    coordinate = forward_kinematics(servo_angle)
    return box_math(coordinate, orientation)
  
  elif state == 6:
    return det_reach(servo_angle)
    
  elif state == 7: # Rerun scanning if no green is found/unable to reach the green box
    int_robot = pos_robot
    pos_robot = extra_robot
    if extra_robot[0] < 90:
      pos_robot[0] = 0.0
      move_robot(serial)
    else:
      pos_robot[0] = 180.0
      move_robot(serial)
    return False

  else:
    pos_robot = robot_pos["check_assembly_position"]
    move_robot(serial)

def det_reach(servo_angle):
  global pos_robot
  initial = np.array(pos_robot)
  current = np.array(servo_angle)
  error = np.abs(initial - current)
  total_error = np.sum(error)
  # print(f"{BLUE}Current angle{RESET}: {servo_angle}")
  # print(f"{BLUE}Target angle{RESET}: {pos_robot}")
  # print(f"Error: {total_error}")
  if total_error <= 5.0:
    print(f"{GREEN}Successfully reach target position{RESET}")
    return True
  else:
    return False

def rotateToAngle(position, serial, speed = [20, 20, 20, 20, 20]): # Rotate all motor to an angle
  set_ch_pos_spd(2, position[0], speed[0], serial)
  set_ch_pos_spd(3, position[1], speed[1], serial)
  set_ch_pos_spd(4, position[2], speed[2], serial)
  set_ch_pos_spd(5, position[3], speed[3], serial)
  set_ch_pos_spd(6, position[4], speed[4], serial)

def speed_equalizer(default_speed = normal_speed):
  # int_robot = initial position
  # pos_robot = target position
  global int_robot, pos_robot
  speed_robot = []
  for i in range(5): # Loop through initial and target servo angle and find the difference
    speed_robot.append(abs(pos_robot[i] - int_robot[i]))

  maximum = max(speed_robot) # Find the maximum difference in angle

  for i in range(5): # Loop through all servo speed
    if speed_robot[i] != 0:
      speed_robot[i] = int((speed_robot[i] / maximum) * default_speed)
      if speed_robot[i] < 5:
        speed_robot[i] = 5
    else:
      speed_robot[i] = 0

  return speed_robot

def box_math(coordinate, orientation = 0):
  # coordinate = coordinate when camera found green box and place the assembly
  # orientation = 0 (box position is horizontal), 1 (box position is vertical)
  global ready_z, place_z1
  y = coordinate[1]
  x = coordinate[0]

  # Calculate the first position to place
  z = math.sqrt((x) * (x) + (y) * (y)) + max_place_dist # length to reach the position of place
  m = (y / x)
  x0 = math.sqrt((z * z) / (1 + ((m) * (m))))
  y0 = (x0) * m

  if orientation == 0:
      m = - m
      dist = min_place_dist
  else:
      dist = 25

  # Calculate the other two position to place
  x1 = math.sqrt((dist * dist) / (1 + ((m) * (m))) )
  x2 = - (x1)
  y1 = (x1) * (m)
  y2 = (x2) * (m)

  if orientation == 0:
    box_pos["box_position1"] = [x0 + x1, y0 + y1, ready_z]
    box_pos["box_position2"] = [x0, y0, ready_z]
    box_pos["box_position3"] = [x0 + x2, y0 + y2, ready_z]
    place_pos["place_position1"] = [x0 + x1, y0 + y1, place_z1]
    place_pos["place_position2"] = [x0, y0, place_z1]
    place_pos["place_position3"] = [x0 + x2, y0 + y2, place_z1]
    box_place_pos["box_position1"] = movePos(x0 + x1, y0 + y1, ready_z)
    box_place_pos["box_position2"] = movePos(x0, y0, ready_z)
    box_place_pos["box_position3"] = movePos(x0 + x2, y0 + y2, ready_z)
    box_place_pos["place_position1"] = movePos(x0 + x1, y0 + y1, place_z1)
    box_place_pos["place_position2"] = movePos(x0, y0, place_z1)
    box_place_pos["place_position3"] = movePos(x0 + x2, y0 + y2, place_z1)
  else:
    box_pos["box_position1"] = [x0, y0, ready_z]
    box_pos["box_position2"] = [x0 + x1, y0 + y1, ready_z]
    box_pos["box_position3"] = [x0 + x2, y0 + y2, ready_z]
    place_pos["place_position1"] = [x0, y0, place_z1]
    place_pos["place_position2"] = [x0 + x1, y0 + y1, place_z1]
    place_pos["place_position3"] = [x0 + x2, y0 + y2, place_z1]
    box_place_pos["box_position1"] = movePos(x0, y0, ready_z)
    box_place_pos["box_position2"] = movePos(x0 + x1, y0 + y1, ready_z)
    box_place_pos["box_position3"] = movePos(x0 + x2, y0 + y2, ready_z)
    box_place_pos["place_position1"] = movePos(x0, y0, place_z1)
    box_place_pos["place_position2"] = movePos(x0 + x1, y0 + y1, place_z1)
    box_place_pos["place_position3"] = movePos(x0 + x2, y0 + y2, place_z1)
    for position in box_pos.values():
      print(position)

  for position in box_pos.values():
    print(f"Box position : {position}")
    if not position:
      return False

  for position in box_place_pos.values():
    print(f"Box position : {position}")
    if not position:
      return False
    
  # for key in box_pos.keys():
  #   print(f"{key}: {box_pos[key]}")

  # for key in place_pos.keys():
  #   print(f"{key}: {place_pos[key]}")

  return True

def gripper_control(direction, serial, default_speed = normal_speed):
  global gripper_state, ready_gripper, close_gripper, normal_speed
  if direction == 1: # close gripper
    gripper_state = 1
    pos_robot[4] = close_gripper
    set_ch_pos_spd(6, close_gripper, default_speed, serial)
  else: # open gripper
    gripper_state = 0
    pos_robot[4] = ready_gripper
    set_ch_pos_spd(6, ready_gripper, default_speed, serial)

def pick(state, pick_position, serial):
  global pos_robot
  if state == 1: # Go to pick position
    key = list(ready_pos.keys())[pick_position - 1]
    pos_robot = movePos(ready_pos[key][0], ready_pos[key][1], ready_pos[key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
  elif state == 2: # Go to assembly position
    key = list(pick_pos.keys())[pick_position - 1]
    pos_robot = movePos(pick_pos[key][0], pick_pos[key][1], pick_pos[key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
  elif state == 3: # Grab the assembly
    gripper_control(1, serial)
  else: # Move back to pick position
    key = list(ready_pos.keys())[pick_position - 1]
    pos_robot = movePos(ready_pos[key][0], ready_pos[key][1], ready_pos[key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)

def place(state, assembly_num, serial):
  global place_z2, pos_robot, extra_robot
  if assembly_num == 6 or assembly_num == 3:
    position = 3
    place_key = list(place_pos.keys())[position - 1]
    if assembly_num == 6:
      place_pos[place_key][2] = place_z2 # Place on top - stacking
  elif assembly_num == 5 or assembly_num == 2:
    position = 2
    place_key = list(place_pos.keys())[position - 1]
    if assembly_num == 5:
      place_pos[place_key][2] = place_z2 # Place on top - stacking
  else:
    position = 1
    place_key = list(place_pos.keys())[position - 1]
    if assembly_num == 4:
      place_pos[place_key][2] = place_z2 # Place on top - stacking

  box_key = list(box_pos.keys())[position - 1]

  if state == 1: # Go to box position
    pos_robot = movePos(box_pos[box_key][0], box_pos[box_key][1], box_pos[box_key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
  elif state == 2: # Place the assembly
    pos_robot = movePos(place_pos[place_key][0], place_pos[place_key][1], place_pos[place_key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
  elif state == 3: # Release the assembly
    gripper_control(0, serial)
  elif state == 4: # Return to previous box position
    pos_robot = movePos(box_pos[box_key][0], box_pos[box_key][1], box_pos[box_key][2])
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
  elif state == 5:
    pos_robot = extra_robot
    move_robot(serial)
  else: # Return to ready position
    pos_robot = robot_pos["check_assembly_position"]
    print(f"{BLUE}Moving to {RESET}{pos_robot}")
    move_robot(serial)
