import pigpio
import time
import os, sys, numpy, argparse
import cv2
import board
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

import prerun_debug_sub
import obj_detection

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
# Global variable
# Parser
output_stat = False 
debug_stat = False

# Serial
serial_stat = False

# Operation
run = True

# Process flag
cam_run = False
led_on = False
first_obj = True
gate_open = False

servo_start_time = None
first_obj_timer = None

# Normal variables
image_center = (320,240)
red_duration = 0.5
led_change = time.time()

in_wait_obj = 0
pick_up_zone_obj = 0
pwm_freq = 50
min_pulse = 500
max_pulse = 2500
wait_time = 3
first_obj_wait = 2

current_num = {
    "assembly" : 0,
    "metal" : 0,
    "plastic" : 0
}

##############################################################
####################### Pin Define ###########################
red_led = 17
green_led = 27 
# obj_signal = 4
obj_signal = 6
gate_servo = 13
ir_sensor = 23


##############################################################
######################### Parser #############################
parser = argparse.ArgumentParser(description="Sub Machine for Object Detection")

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
######################### PiGPIO #############################
pi = pigpio.pi()
if debug_stat:
    prerun_debug_sub.debug_daemon(pi)
else:
    if not pi.connected:
        print(f"{RED}ERROR{RESET}: Failed to connect to pigpiod daemon")
        sys.exit(1)

##############################################################
####################### Main code ############################

# Setup pins
def setup():
    try:
        print(f"{YELLOW}INFO{RESET}: Setting up GPIO")
        pi.set_mode(red_led, pigpio.OUTPUT)
        pi.set_mode(green_led, pigpio.OUTPUT)
        pi.set_mode(obj_signal, pigpio.INPUT)
        pi.set_mode(ir_sensor, pigpio.INPUT)
        
        pi.set_pull_up_down(obj_signal, pigpio.PUD_DOWN)
        pi.set_glitch_filter(obj_signal, 10000)

        pi.set_pull_up_down(ir_sensor, pigpio.PUD_UP)
        pi.set_glitch_filter(ir_sensor, 1000)
        print(f"{GREEN}STATUS{RESET}: GPIO set up successfully")
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up GPIO - {e}")
        sys.exit(1)

# Setup serial
def serial_setup():
    global serial_stat
    try:
        ser = serial.Serial(ser_subm_addr, 9600, timeout=1)
        if debug_stat:
            prerun_debug_sub.debug_serial(ser, ser_subm_addr)
            prerun_debug_sub.debug_uart(ser)
        print(f"{GREEN}STATUS{RESET}: Serial connection set up successfully")
        serial_stat = True
        return ser
    
    except serial.SerialException as e:
        print(f"{RED}ERROR{RESET}: Failed to set up serial connection - {e}")
        sys.exit(1)

# Gate servo angle
def set_angle(angle):
    global gate_open, servo_start_time
    try:
        if not 0 <= angle <= 180:
            print(f"{RED}ERROR{RESET}: Roptation angle must between 0 and 180")
            return
        
        print(f"{YELLOW}INFO{RESET}: Turning gate servo")
        pulse_width = min_pulse + (max_pulse - min_pulse) * angle / 180
        pi.set_servo_pulsewidth(gate_servo, pulse_width)

        if angle == 90:
            gate_open = True
            servo_start_time = time.time()
            print(f"{GREEN}STATUS{RESET}: Gate opened")
        else:
            gate_open = False
            print(f"{GREEN}STATUS{RESET}Gate closed")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Error in setting angle for gate servo - {e}")

# Gate servo initial
def gate_servo_initial():
    global gate_open
    try:
        print(f"{YELLOW}INFO{RESET}: Setting up gate servo's pwm and initial position")
        pi.set_PWM_frequency(gate_servo, pwm_freq)
        set_angle(90)
        gate_open = True
        print(f"{GREEN}STATUS{RESET}: Gate servo initialization completed")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Error setting up gate servo - {e}")

# Initialize camera
def cam_setup():
    try:
        if debug_stat:
            prerun_debug_sub.debug_picam()
        picam2.configure(picam2.create_preview_configuration(main={"size":(640,480)}))
        picam2.start()
        print(f"{GREEN}STATUS{RESET}: PiCam set up successfully")

    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up camera - {e}")
        sys.exit(1)

def cam_display(frame):
    try:
        # Display
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("Camera View", frame)
        key = cv2.waitKey(1)
        
        return key
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to display camera view window - {e}")
        sys.exit(1)

# Callback for object count
def signal_callback(gpio, level, tick):
    global in_wait_obj
    if level == 1:
        in_wait_obj += 1
        if debug_stat:
            print(f"{BLUE}DEBUG{RESET}: Edge detected in pin GPIO {obj_signal}: HIGH at tick {tick}\n")

# Callback for infrared sensor
def ir_callback(gpio, level, tick):
    global in_wait_obj, pick_up_zone_obj, first_obj
    if first_obj:
        if in_wait_obj > 0 and level == 0:
            set_angle(0)
            in_wait_obj -= 1
            pick_up_zone_obj += 1
            print(f"{GREEN}STATUS{RESET}: First assembly released to pick up zone")
            if debug_stat:
                print(f"{BLUE}DEBUG{RESET}: Edge detected in pin GPIO {ir_sensor}: HIGH at tick {tick}\n")
    else:
        if level == 0 and gate_open and in_wait_obj > 0:
            set_angle(0)
            in_wait_obj -= 1
            pick_up_zone_obj += 1
            print(f"{GREEN}STATUS{RESET}: 1 assembly released to pick up zone")
            if debug_stat:
                print(f"{BLUE}DEBUG{RESET}: Edge detected in pin GPIO {ir_sensor}: HIGH at tick {tick}\n")

# Main
def main():
    global cam_run, led_on, led_change, first_obj, in_wait_obj, run, first_obj_timer, pick_up_zone_obj, gate_open
    setup()
    ser = serial_setup()
    gate_servo_initial()
    
    # Edge detection callback
    try:
        callback_obj = pi.callback(obj_signal, pigpio.RISING_EDGE, signal_callback)
        callback_ir = pi.callback(ir_sensor, pigpio.FALLING_EDGE, ir_callback)
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Failed to set up edge detection - {e}")
        sys.exit(1)
    
    try:  
        while True:
            # Activate process
            if run:
                if not cam_run:
                    cam_setup()
                    cam_run = True  
                frame = picam2.capture_array()
                led_current_time = time.time()
                servo_current_time = time.time()

                # Object detection
                frame, a_num, m_num, p_num = obj_detection.detect_obj(frame)

                current_num["assembly"] = a_num
                current_num["metal"] = m_num
                current_num["plastic"] = p_num
                print(f"{GREEN}STATUS{RESET}: Assembly - {a_num} | Metal - {m_num} | Plastic - {p_num}")

                if current_num["metal"] > 0 or current_num["plastic"] > 0:
                    if (led_current_time - led_change) > red_duration:
                        if led_on:
                            pi.write(red_led, 0)
                            led_on = False
                        else:
                            pi.write(green_led, 0)
                            pi.write(red_led, 1)
                            led_on = True
                        led_change = led_current_time
                    
                    if current_num["metal"] > 0 and current_num["plastic"] == 0:
                        false_object = f"Metal - {current_num['metal']}"
                    elif current_num["metal"] == 0 and current_num["plastic"] > 0:
                        false_object = f"Plastic - {current_num['plastic']}"
                    else:
                        false_object = f"Metal - {current_num['metal']} | Plastic - {current_num['plastic']}"
                    
                    
                    print(f"{RED}ERROR{RESET}: False object on slider - <{false_object}>")
                else:
                    pi.write(red_led, 0)
                    pi.write(green_led, 1)

                # Serial response and feedbacks
                if ser.in_waiting > 0:
                    line = ser.readline()
                    response = line.decode('utf-8').strip()
                    print(line.hex())
                    # Send number of each classes
                    if response == "Send results":
                        print(f"\n{YELLOW}INFO{RESET}: Request received from Master Machine - Class count")
                        feedback = f"A:{current_num['assembly']},M:{current_num['metal']},P:{current_num['plastic']}\n"
                    # Send number of in-wait objects and in pick up zone
                    elif response == "Obj count":
                        print(f"\n{YELLOW}INFO{RESET}: Request received from Master Machine - Object count")
                        feedback = f"P:{pick_up_zone_obj},W:{in_wait_obj}\n"
                    # Terminate Sub Machine process
                    elif response == "Terminate process":
                        print(f"\n{YELLOW}INFO{RESET}: Request received from Master Machine - Terminate process")
                        feedback = f"end\n"
                        run = False
                    # Deduct pick up zone count
                    elif response == "Deduct count":
                        print(f"\n{YELLOW}INFO{RESET}: Request received from Master Machine - Deduct count")
                        pick_up_zone_obj -= 1
                        feedback = f"D:{pick_up_zone_obj}\n"
                    else:
                        print(f"{RED}ERROR{RESET}: Unrecognized request from Master Machine")
                        feedback = f"UKN:{response}\n"
                    
                    if debug_stat:
                        print(f"{BLUE}DEBUG{RESET}: Message sent - {feedback}")
                    ser.write(feedback.encode('utf-8'))
                    print(f"{YELLOW}INFO{RESET}: Feedback sent to Master Machine")
                else:
                    print(f"{YELLOW}INFO{RESET}: No request from Master Machine\n")
                
                # In wait object count
                if first_obj:
                    if debug_stat:
                        print(f"{BLUE}DEBUG{RESET}: Waiting for first object")
                    
                    if not gate_open:
                        first_obj = False
                        print(f"{GREEN}STATUS{RESET}: First object enters pick up zone")
                    # if in_wait_obj >= 1 and first_obj_timer is None:
                    #     print(f"{GREEN}STATUS{RESET}: First object passed through")
                    #     first_obj_timer = time.time()
                    
                    # if first_obj_timer is not None and (servo_current_time - first_obj_timer) >= first_obj_wait:
                    #     set_angle(0)
                    #     in_wait_obj -= 1
                    #     pick_up_zone_obj += 1
                    #     first_obj = False
                    #     first_obj_timer = None
                        
                else:
                    total_obj = current_num["assembly"] + current_num["metal"] + current_num["plastic"]
                    false_obj = current_num["metal"] + current_num["plastic"]
                    if pick_up_zone_obj == 0:
                        if total_obj == 0:
                            print(f"{YELLOW}INFO{RESET}: Pick up zone is empty. Releasing assembly\n")
                            set_angle(90)
                        elif total_obj > 0 and false_obj == 0:
                            print(f"{RED}ERROR{RESET}: Mismatched number of objects in pick up zone. Rechecking number of objects\n")              
                    else:
                        if total_obj == pick_up_zone_obj and false_obj == 0:
                            print(f"{YELLOW}INFO{RESET}: Pick up zone occupied")
                        elif total_obj != pick_up_zone_obj and false_obj == 0:
                            print(f"{RED}ERROR{RESET}: Mismatched number of objects in pick up zone. Rechecking number of objects\n")
                        
                        if in_wait_obj != 0:
                            print(f"{YELLOW}INFO{RESET}: Objects in-wait in slider")
                        else:
                            print(f"{YELLOW}INFO{RESET}: No in wait objects")
                    
                    if false_obj != 0:
                        print(f"{RED}ERROR{RESET}: False objects at pick up zone. Clear them before proceeding\n")
                    # if gate_open and servo_start_time is not None:
                    #     if (servo_current_time - servo_start_time) > wait_time:
                    #         set_angle(0)
                    #         pick_up_zone_obj = min(3, in_wait_obj)
                    #         in_wait_obj -= pick_up_zone_obj
                    #         print(f"{YELLOW}INFO{RESET}: {pick_up_zone_obj} objects released to pick up zone\n")
                
                if debug_stat:
                    print(f"{BLUE}DEBUG{RESET}: Current number of objects in pick up zone - {pick_up_zone_obj}")
                    print(f"{BLUE}DEBUG{RESET}: Current number of objects in wait - {in_wait_obj}")
                
                # Cam frame
                key = cam_display(frame)

                # Terminate event
                if key == 27:
                    print(f"\n{YELLOW}INFO{RESET}: Program terminated by user")
                    break
            
            # End process
            else:
                print(f"{YELLOW}INFO{RESET}: Process terminated")
                break

    except KeyboardInterrupt:
        print(f"{YELLOW}\nINFO{RESET}: Force termination ")

    except Exception as e:
        print(f"{RED}Error{RESET}: {e}")
    
    finally:  
        if cam_run:     
            picam2.stop()
        if serial_stat:
            ser.close()
        callback_ir.cancel()
        callback_obj.cancel()
        cv2.destroyAllWindows()
        pi.write(red_led, 0)
        pi.write(green_led, 0)
        pi.set_glitch_filter(obj_signal, 0)
        pi.set_servo_pulsewidth(gate_servo, 0)
        pi.stop()     
        print(f"{GREEN}STATUS{RESET}: System cleaned up successfully")

        if output_stat:
            sys.stdout.file.close()
            sys.stdout = sys.stdout.stdout
            print(f"{YELLOW}INFO{RESET}: output.log file saved to <{file_path}>")




if __name__ == "__main__":
    main()