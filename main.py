import pigpio
import time
import os, sys, numpy, argparse
import cv2
import board
import adafruit_vl53l1x
from picamera2 import Picamera2
import serial

# Colour code
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
RESET="\033[0m"

# Directory
file_path = os.path.dirname(os.path.abspath(__file__))
output_file = f"{file_path}/output.log"

# Import sub-files
sys.path.append(file_path)

import prerun_debug
import detect_green_box
import request_sub_board
import mark_grid
import assembly_position
import motion

##############################################################
######################## Logging #############################
class Tee:
    def __init__(self, filename):
        self.file = open(filename, "w")
        self.stdout = sys.stdout
    
    def write(self, message):
        self.stdout.write(message)
        self.file.write(message)

    def flush(self):
        self.stdout.flush()
        self.file.flush()

##############################################################
####################### Variables ############################
picam2 = Picamera2()
ser_subm_addr = "/dev/ttyAMA3"
ser_sc08_addr = "/dev/ttyAMA4"
# Global variable
# Parser
output_stat = False 
debug_stat = False

# ToF
sensor_stat = False

# Serial
serial_stat = False
serial_sc_stat = False

# Robot arm operation
run = False
plc_end = False
robot_terminate = False
end_stat = None
first_loop = True
cam_run = False
arm_setup = False

# Process flag
get_box = False
grid_stat = False
motor_move = False

# Get box
x_pos = False
y_pos = False
box_move = False
timer = 0
reach_state = False

# Pick and place
pick_state = False
place_state = False
pick_place_stage = 1
picked_num = 0 # Number of assembly picked
total_obj = 0 # Signal from sub board

# Normal variables
image_center = (320,240)

count_box_x = 0
count_box_y = 0
position = None
box = {
    "width" : [],
    "height" : []
}

initial_pos = {
    "channel_1" : 90,
    "channel_2" : 177,
    "channel_3" : 91,
    "channel_4" : 93
}

pick_up_zone_obj = 0

detect_loop = 0
best_ind = 0
r_max_obj_ind = []
r_distance = []

##############################################################
####################### Pin Define ###########################
ict_state = 17

##############################################################
######################### Parser #############################
parser = argparse.ArgumentParser(description="Robot Arm")

parser.add_argument('-s', '--save', type=str, default="off", help="Save results in output.log file (On/Off)")
parser.add_argument('-d', '--debug', type=str, default="off", help="Enable debug features (On/Off)")

args = parser.parse_args()

save_output = args.save.strip().upper()
debug = args.debug.strip().upper()

if save_output == "ON":
    if os.path.exists(output_file):
        os.remove(output_file)
    sys.stdout = Tee(output_file)
    sys.stderr = sys.stdout
    output_stat = True
    print(f"{YELLOW}INFO{RESET}: Output messages saved to output.log file\n")
else:
    print(f"{YELLOW}INFO{RESET}: Output messages will NOT be saved to output.log file\n")

if debug == "ON":
    debug_stat = True
    print(f"{YELLOW}INFO{RESET}: Debug mode turned ON\n")

##############################################################
######################### PiGPIO ############################
pi = pigpio.pi()
if debug_stat:
    prerun_debug.debug_daemon(pi)
    prerun_debug.debug_i2C()
else:
    if not pi.connected:
        print(f"{RED}ERROR{RESET}: Failed to connect to pigpiod daemon")
        sys.exit(1)

##############################################################
####################### Main code ############################

# Setup pins
def setup():
    try:
        # Plc status
        pi.set_mode(ict_state, pigpio.INPUT)
        pi.set_pull_up_down(ict_state, pigpio.PUD_DOWN)
        # Pressure sensor
        # pi.set_mode(pressure_1, pigpio.INPUT)
        # pi.set_mode(pressure_2, pigpio.INPUT)
        print(f"{GREEN}STATUS{RESET}: GPIO set up successfully")

    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up GPIO - {e}")
        sys.exit(1)

# Setup ToF sensor
def setup_tof():
    global sensor_stat
    try:
        i2c = board.I2C()
        sensor = adafruit_vl53l1x.VL53L1X(i2c)
        sensor.start_ranging()
        if debug_stat:
            prerun_debug.debug_tof(sensor)
        print(f"{GREEN}STATUS{RESET}: VL53L1X set up successfully")       
        sensor_stat = True
        return sensor
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up VL53L1X - {e}")

# Setup serial
def serial_setup():
    global serial_stat, serial_sc_stat
    try:
        ser = serial.Serial(ser_subm_addr, 9600, timeout=1)
        ser_sc = serial.Serial(ser_sc08_addr, 9600, timeout=1)
        if debug_stat:
            print(f"{BLUE}DEBUG{RESET}: Debug for Sub Machine Serial & UART")
            prerun_debug.debug_serial(ser, ser_subm_addr)
            prerun_debug.debug_uart(ser)
            print(f"{BLUE}DEBUG{RESET}: Debug for SC08 Serial & UART")
            prerun_debug.debug_serial(ser_sc, ser_sc08_addr)
            prerun_debug.debug_sc08(ser_sc)
        print(f"{GREEN}STATUS{RESET}: Serial connection set up successfully for Sub Machine and sc08")
        serial_stat = True
        serial_sc_stat = True
        return ser, ser_sc
    
    except serial.SerialException as e:
        print(f"{RED}ERROR{RESET}: Failed to set up serial connection - {e}")
        sys.exit(1)

# Check ict signal
def ict_signal(gpio, level, tick):
    global run, plc_end
    if level == 1:
        print(f"\n{YELLOW}INFO{RESET}: PLC Operation Start")
        print(f"{YELLOW}INFO{RESET}: Activate Robot Arm")
        run = True      
    else:
        print(f"\n{YELLOW}INFO{RESET}: PLC Operation Stopped")
        plc_end = True

# Initialize camera
def cam_setup():
    try:
        if debug_stat:
            prerun_debug.debug_picam()
        picam2.preview_configuration.main.size = (640, 480)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()
        print(f"{GREEN}STATUS{RESET}: PiCam set up successfully")

    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up camera - {e}")
        sys.exit(1)

def cam_display(frame):
    try:
        # Draw center
        cv2.circle(frame, image_center, 5, (255, 0, 0), 2)  # Blue color, 5px radius
        cv2.line(frame, (image_center[0] - 10, image_center[1]), (image_center[0] + 10, image_center[1]), (255, 0, 0), 2)
        cv2.line(frame, (image_center[0], image_center[1] - 10), (image_center[0], image_center[1] + 10), (255, 0, 0), 2)
        
        # Display
        cv2.imshow("Camera View", frame)
        key = cv2.waitKey(1)
        
        return key
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to display camera view window - {e}")
        sys.exit(1)

# Check object class
def slider_class(ser):
    try:
        if serial_stat:
            print(f"{YELLOW}INFO{RESET}: Sending requests to Sub Machine")
            a_num, m_num, p_num = request_sub_board.request_num(ser)
            if a_num == -1 or m_num == -1 or p_num == -1:
                print(f"{RED}ERROR{RESET}: Requested values error. Check UART connection")
            else:
                print(f"{YELLOW}INFO{RESET}: Response received from Sub Machine")
                print(f"{GREEN}STATUS{RESET}: Assembly - {a_num} | Metal - {m_num} | Plastic - {p_num}")
            
            if m_num > 0 or p_num > 0:
                if m_num > 0 and p_num == 0:
                    false_object = f"Metal - {m_num}"
                elif m_num == 0 and p_num > 0:
                    false_object = f"Plastic - {p_num}"
                else:
                    false_object = f"Metal - {m_num} | Plastic - {p_num}"
                print(f"{RED}ERROR{RESET}: False object on slider - <{false_object}>")
            
            return a_num, m_num, p_num
        else:
            print(f"{RED}ERROR{RESET}: Error in Sub Machine UART connection. Check UART connection")
            return -1, -1, -1
    
    except serial.SerialException as se:
        print(f"{RED}ERROR{RESET}: Unexpected serial error - {se}")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Error requesting classes counts fron Sub Machine - {e}")

# Box position
def box_position(frame, ser_sc):
    global x_pos, y_pos, count_box_x, count_box_y, box, get_box, position, box_move, timer, reach_state, pick_place_stage
    angle_x_box, angle_y_box, width, height = None, None, 0, 0

    if not x_pos or not y_pos:
        try:
            print(f"{YELLOW}INFO{RESET}: Searching for box location")
            frame, hsv, angle_x_box, angle_y_box, width, height = detect_green_box.detect_box(frame)
        
        except Exception as e:
            print(f"{RED}ERROR{RESET}: Error detecting green box - {e}")
    
    print(angle_x_box, angle_y_box)
    # Check x-axis position
    if not x_pos and not y_pos:
        print(f"{YELLOW}INFO{RESET}: Checking x-axis position")
        if angle_x_box is not None:
            if angle_x_box < -1:
                count_box_x += 1
                box["width"].append(width)
                box["height"].append(height)
                # if count_box_x >= 3:
                x_pos = True
                print(f"{GREEN}STATUS{RESET}: Detection of x-axis position completed")
                motion.scan_box(2, ser_sc)
                timer = time.time()
            else:
                count_box_x = 0
        else:
            rescan = motion.scan_box(1, ser_sc) # Swap between 0 and 180 degress
            if rescan:
                timer = time.time()
    # Check y-axis position
    if x_pos and not y_pos and (time.time() - timer > 3.0):
        print(f"{YELLOW}INFO{RESET}: Checking y-axis position")
        if not box_move: # Move towards green box
            motion.scan_box(3, ser_sc)
            box_move = True
        else:
            if angle_y_box is not None:
                print("Able to have angle_y_box")
                if angle_y_box >= 0:
                    # count_box_y += 1
                    box["width"].append(width)
                    box["height"].append(height)
                    # if count_box_y >= 10:
                    y_pos = True
                    print(f"{GREEN}STATUS{RESET}: Detection of y-axis position completed")
                    motion.scan_box(4, ser_sc)
                # else:
                #     count_box_y = 0
            else:
                result = motion.scan_box(6, ser_sc)
                if result: # Reach the maximum extension but no green box found
                    print(f"{BLUE}No green box found{RESET}")
                    x_pos = False
                    box_move = motion.scan_box(7, ser_sc)
                    timer = time.time()
                elif not box_move: # Unable to reach the other part of the 
                    x_pos = False
                    box_move = motion.scan_box(7, ser_sc)
                    timer = time.time()

    if debug_stat:
        print(f"{BLUE}DEBUG{RESET}: Angle_x: {angle_x_box:.4f} | Angle_y: {angle_y_box:.4f}")
        print(f"{BLUE}DEBUG{RESET}: Width - {width} | Height - {height}")

    # Check box orientation
    if x_pos and y_pos:
        avg_width, avg_height = 0, 0
        avg_width = round(sum(box["width"])/len(box["width"]))
        avg_height = round(sum(box["height"])/len(box["height"]))
        print(avg_width, avg_height)
        print(f"{YELLOW}STATUS{RESET}: Checking orientation of box")

        if debug_stat:
            print(f"\n{BLUE}DEBUG{RESET}: Width and height values:")
            for keys, values in box.items():
                print(f"{BLUE}DEBUG{RESET}: {keys} - {values}")
            print(f"{BLUE}DEBUG{RESET}: Average width - {avg_width}")
            print(f"{BLUE}DEBUG{RESET}: Average height - {avg_height}\n")
        
        if avg_width > avg_height:
            box_move = motion.scan_box(5, ser_sc, orientation = 0)
            if not box_move:
                print(f"{RED}ERROR{RESET}: Error reaching the box. Restarting process")
                x_pos, y_pos = False, False
                count_box_x, count_box_y = 0, 0
                box["width"], box["height"] = [], []
                box_move = motion.scan_box(7, ser_sc)
            else:
                position = "horizontal"
                print(f"{GREEN}STATUS{RESET}: Orientation of box is {position}")
                get_box = True
                print(f"{GREEN}STATUS{RESET}: Green box positioning setup completed. Assembly transferring process starts\n")
                pick_place_stage = 0
                motion.scan_box(8, ser_sc)
            timer = time.time()
        elif avg_width < avg_height:
            box_move = motion.scan_box(5, ser_sc, orientation = 0)
            if not box_move:
                print(f"{RED}ERROR{RESET}: Error reaching the box. Restarting process")
                x_pos, y_pos = False, False
                count_box_x, count_box_y = 0, 0
                box["width"], box["height"] = [], []
                box_move = motion.scan_box(7, ser_sc)
            else:
                position = "vertical"
                print(f"{GREEN}STATUS{RESET}: Orientation of box is {position}")
                get_box = True
                print(f"{GREEN}STATUS{RESET}: Green box positioning setup completed. Assembly transferring process starts\n")
                pick_place_stage = 0
                motion.scan_box(8, ser_sc)
            timer = time.time()
        else:
            print(f"{RED}ERROR{RESET}: Error determining orientation of box. Restarting process")
            x_pos, y_pos = False, False
            count_box_x, count_box_y = 0, 0
            box["width"], box["height"] = [], []
            result = motion.scan_box(6, ser_sc)
            if result: # Reach the maximum extension but no green box found
                box_move = motion.scan_box(7, ser_sc)
                timer = time.time()
            print(f"{YELLOW}STATUS{RESET}: Process restarted\n")
        
    # cv2.setMouseCallback("Camera View", detect_green_box.mouse_callback, param=hsv)

# Turn off sub machine
def end_sub_machine(ser):
    global end_stat

    end_stat = request_sub_board.end_process(ser)
    if end_stat is not None:
        if end_stat == 1:
            print(f"{YELLOW}INFO{RESET}: Sub Board process ended successfully")
        else:
            print(f"{RED}ERROR{RESET}: Unexpected response from Sub Board")
    else:
        print(f"{RED}ERROR{RESET}: No response from Sub Board. Check UART connection")

# Main
def main():
    global run, cam_run, grid_stat, motor_move, detect_loop, robot_terminate, first_loop, timer, arm_setup, pick_state, total_obj
    global place_state, pick_place_stage, picked_num
    setup()
    sensor = setup_tof()
    ser, ser_sc = serial_setup()
    arm_setup = motion.robot_arm_setup(1, ser_sc)
    
    if not arm_setup:
        print(f"{RED}ERROR{RESET}: Unable to setup robot arm")
        print(f"{YELLOW}INFO{RESET}: Possible error - Math error or not logical position")
        sys.exit(1)

    # Edge detection callback
    try:
        cb = pi.callback(ict_state, pigpio.EITHER_EDGE, ict_signal)
        print(f"{GREEN}STATUS{RESET}: Edge detection set up successfully")
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up edge detection - {e}")
        sys.exit(1)

    try:
        while True:
            # Turn off
            # Add one more complete flag for placing
            if plc_end and not motor_move: # and complete flag
                total_obj, in_wait_obj, a_num, m_num, p_num = 0, 0, 0, 0, 0

                print(f"{YELLOW}INFO{RESET}: PLC process terminated")
                total_obj, in_wait_obj = request_sub_board.request_count(ser)
                a_num, m_num, p_num = slider_class(ser)
                if debug_stat:
                    print(f"{BLUE}DEBUG{RESET}: Number of objects on pick up zone - {total_obj}")
                    print(f"{BLUE}DEBUG{RESET}: Number of objects in-wait - {in_wait_obj}")
                
                # False object on track
                if m_num != 0 or p_num != 0:
                    print(f"{RED}ERROR{RESET}: False object on pick up zone. Robot arm process terminated")
                    robot_terminate = True
                elif a_num == -1 or m_num == -1 or p_num == -1:
                    print(f"{RED}ERROR{RESET}: Error requesting number of class. Robot arm process terminated")
                    robot_terminate = True
                # Assembly only
                else:            
                    if total_obj != 0 or in_wait_obj != 0:
                        # complete flag
                        print(f"{YELLOW}INFO{RESET}: Remaining assembly on track and pick up zone")
                        print(f"{YELLOW}INFO{RESET}: Robot arm process continues")
                        continue
                    # Pick up zone and track empty
                    else:
                        print(f"{YELLOW}INFO{RESET}: All assembly in-wait and on pick up zone cleared")
                        robot_terminate = True
            
            if robot_terminate:
                print(f"{YELLOW}INFO{RESET}: Deactivate robot arm")
                run = False
                end_sub_machine(ser)
                break

            # elif not arm_setup:
            #     print(f"{RED}Robot arm failed to setup {RESET}")

            # Turn on
            elif run:
                if first_loop:
                    print(f"{GREEN}STATUS{RESET}: Robot arm activated successfully\n")
                    first_loop = False
                
                if not cam_run:
                    cam_setup()
                    cam_run = True

                # Capture frame
                frame = picam2.capture_array()
                
                # Mark grid
                if not grid_stat:
                    servo_angle = motion.check(ser_sc)
                    reach_state = motion.det_reach(servo_angle)
                    if reach_state:
                        print(f"{YELLOW}INFO{RESET}: Marking grid for end of slider")
                        frame, grid, captured = mark_grid.get_grid(frame)

                        if debug_stat:
                            print(f"{BLUE}DEBUG{RESET}: Current grid information:")
                            for key, values in grid.items():
                                print(f"{BLUE}DEBUG{RESET}: {key} - {values}")
                            print(f"{BLUE}DEBUG{RESET}: Capturing grid on next frame\n")
                        
                        if captured:
                            print(f"{GREEN}STATUS{RESET}: Grid information captured and stored")
                            if debug_stat:
                                print(f"{BLUE}DEBUG{RESET}: Current grid information:")
                                for key, values in grid.items():
                                    print(f"{BLUE}DEBUG{RESET}: {key} - {values}")
                                print(f"{BLUE}DEBUG{RESET}: Grid marking process completed\n")
                            reach_state = False
                            grid_stat = True
                            motion.robot_arm_setup(2, ser_sc)
                            timer = time.time()
                
                # Detect box
                if not get_box and grid_stat:
                    if (time.time() - timer > 3):
                        box_position(frame, ser_sc)
                
                # distance = tof.check_distance(sensor)
                # print(f"Distance: {distance}mm")

                # Get signal from sub board
                if pick_place_stage == 0:
                  servo_angle = motion.check(ser_sc)
                  reach_state = motion.det_reach(servo_angle)
                  if (time.time() - timer > 5.5) and reach_state:
                    total_obj, in_wait_obj = request_sub_board.request_count(ser)

                # servo_angle = motion.check(ser_sc)
                # print(f"Ready: {motion.det_reach(servo_angle)}")

                # Detect obejct
                if get_box and not motor_move and grid_stat and (total_obj >= 1):
                    # Check slider
                    # a_num, m_num, p_num = slider_class(ser)
                    pick_place_stage = 1

                    # Detect position
                    print(f"{YELLOW}INFO{RESET}: Detecting position of assembly on slider")
                    if detect_loop < 20:
                        print(f"{YELLOW}INFO{RESET}: Detection {detect_loop+1}")
                        frame, max_obj_ind, dist = assembly_position.detect_assembly_position(frame)

                        if max_obj_ind != 0 and len(dist) != 0:
                            r_max_obj_ind.append(max_obj_ind)
                            r_distance.append(dist)
                            detect_loop += 1

                        if debug_stat:
                            print(f"{BLUE}DEBUG{RESET}: Number of assembly detected - {max_obj_ind}")
                            print(f"{BLUE}DEBUG{RESET}: Distance of each object from bottom of frame (ascending) - {dist}")
                        
                        print(f"{GREEN}STATUS{RESET}: Detection {detect_loop} complete\n")
                    
                    if detect_loop >= 20:
                        print(f"{YELLOW}INFO{RESET}: Detection complete, analyzing results")
                        final_ind, final_dist = assembly_position.compare_results(r_max_obj_ind, r_distance, best_ind)
                        pos, grid_num = assembly_position.locate_grid(final_ind, final_dist, grid)

                        # if debug_stat:
                        print(f"{BLUE}DEBUG{RESET}: Final assembly detected - {final_ind}")
                        print(f"{BLUE}DEBUG{RESET}: Final distance detected - {final_dist}")
                        print(pos, grid_num)
                        
                        if pos == -1 or grid_num == -1:
                            print(f"{RED}ERROR{RESET}: Unexpected position or unrecognized grid. Check location of assembly")
                            print(f"{YELLOW}INFO{RESET}: Restarting detection")
                            detect_loop = 0
                        else:
                            print(f"{GREEN}STATUS{RESET}: Assembly located at {grid_num} with distance {pos} pixels from bottom")
                            detect_loop = 0
                            motor_move = True
                            pick_state = True
                            timer = time.time()
                            print(f"{YELLOW}INFO{RESET}: Preparing to grip the assembly to complete box")

                if motor_move and pick_state: # pick when assembly drops to pick region
                   pick_position = int(grid_num.split('_')[1])
                   angle = motion.check(ser_sc)
                   reach_state = motion.det_reach(angle)
                   if (time.time() - timer > 3.0) and reach_state:
                    print(f"{YELLOW}Pick stage{RESET} {pick_place_stage}")
                    motion.pick(pick_place_stage, pick_position, ser_sc)
                    pick_place_stage += 1
                    reach_state = False
                    timer = time.time()
                    if (pick_place_stage >= 5):
                        pick_place_stage = 1
                        picked_num += 1
                        pick_state = False
                        place_state = True

                if motor_move and place_state:
                    angle = motion.check(ser_sc)
                    print(angle)
                    reach_state = motion.det_reach(angle)
                    print(f"Ready: {reach_state}")
                    if (time.time() - timer > 3.0) and reach_state:
                        print(f"Stage: {pick_place_stage}")
                        motion.place(pick_place_stage, picked_num, ser_sc)
                        pick_place_stage += 1
                        reach_state = False
                        timer = time.time()
                        if (pick_place_stage > 6):
                            print(f"{GREEN}Successfully pick and place one assembly{RESET}")
                            print(f"{BLUE}Assembly in complete box:{RESET} {picked_num}")
                            pick_place_stage = 0
                            place_state = False
                            motor_move = False # After robot arm return to ready position
                            total_obj = 0
                            pick_up_zone_obj = request_sub_board.deduct_count(ser)
                            # Global variable to return to request signal from sub board
                            if picked_num == 6:
                                print(f"Please {YELLOW}clear{RESET} the {GREEN}GREEN BOX{RESET}.")
                                print(f"{YELLOW}[ALERT]{RESET}Total number of assembly in complete box is reset")
                                picked_num = 0

                key = cam_display(frame)          

                # Terminate event
                if key == 27:
                    print(f"\n{YELLOW}INFO{RESET}: Program terminated by user")
                    end_sub_machine(ser)
                    break
            
            # Wait for response
            else:
                print(f"{YELLOW}INFO{RESET}: Waiting Signal From PLC")
                time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"{YELLOW}\nINFO{RESET}: Force termination ")

    except serial.SerialException as es:
        print(f"{RED}Error{RESET}: {es}")
    
    except Exception as e:
        print(f"{RED}Error{RESET}: {e}")
    
    finally:
        if sensor_stat:       
            sensor.stop_ranging()
        if cam_run:
            picam2.stop()
        if serial_stat:
            ser.close()
        if serial_sc_stat:
            ser_sc.close()
        cv2.destroyAllWindows()
        pi.stop()
        print(f"{GREEN}STATUS{RESET}: System cleaned up successfully")

        if output_stat:
            sys.stdout.file.close()
            sys.stdout = sys.stdout.stdout
            print(f"{YELLOW}INFO{RESET}: output.log file saved to <{file_path}>")




if __name__ == "__main__":
    main()