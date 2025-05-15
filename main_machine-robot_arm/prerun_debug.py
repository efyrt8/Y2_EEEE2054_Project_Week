# ONLY ENABLED WHEN DEBUG MODE IS SET TO ON
# TO SET DEBUG MODE TO ON, USE "-d on" WHEN RUNNING "main.py"

import pigpio
import time
import subprocess, sys, os
import serial

import tof

# Colour code
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
RESET="\033[0m"

pins_to_test = [6, 12, 13, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]

# Check pigpio daemon
def callback_test(pin, level, tick):
    print(f"{BLUE}DEBUG{RESET}: Edge detected in pin GPIO {pin}: {'HIGH' if level==1 else 'LOW'} at tick {tick}")

def debug_daemon(pi):
    print(f"{BLUE}DEBUG{RESET}: Checking pigpiod daemon status...")
    process = subprocess.Popen(['sudo', 'systemctl', 'status', 'pigpiod'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    print(f"{BLUE}DEBUG{RESET}: Status of pigpiod:")
    for line in process.stdout:
        print(line, end="")

    process.wait()
    if process.returncode != 0:
        print(f"{RED}ERROR{RESET}: Failed to check pigpiod status - Command returned at exitcode {process.returncode}")
        sys.exit(1)

    if not pi.connected:
        print(f"{RED}ERROR{RESET}: Failed to connect to pigpio daemon.")
        
    try: 
        print(f"{BLUE}DEBUG{RESET}: Version of pigpio: {pi.get_pigpio_version}")

        callbacks = []
        for pin in pins_to_test:
            print(f"\n{BLUE}DEBUG{RESET}: Testing GPIO {pin}")
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            level = pi.read(pin)
            print(f"{BLUE}DEBUG{RESET}: Initial state of GPIO {pin} - {'LOW' if pi.read(pin)==0 else 'HIGH'}")

            try:
                cb = pi.callback(pin, pigpio.EITHER_EDGE, callback_test)
                callbacks.append(cb)
                print(f"{BLUE}DEBUG{RESET}: Edge detection added successfully on GPIO {pin}")
            except Exception as e:
                print(f"{BLUE}DEBUG{RESET}: Failed to add edge detection on GPIO {pin} - {e}")
        
        print(f"\n{BLUE}DEBUG{RESET}: GPIO test completed. Proceeding to next command\n")
        time.sleep(1)

    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected Error - {e}")
    
    finally:
        for cb in callbacks:
            cb.cancel()
        print(f"{BLUE}DEBUG{RESET}: Callbacks in debug terminated\n")

# Check i2c
def debug_i2C():
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking I2C status...")
        i2c_enabled = os.path.exists("/dev/i2c-1")
        print(f"{BLUE}DEBUG{RESET}: I2C enabled - {i2c_enabled}")

        if i2c_enabled:
            print(f"{BLUE}DEBUG{RESET}: Scanning i2c devices")
            process = subprocess.Popen(['sudo', 'i2cdetect', '-y', '1'],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            
            print(f"{BLUE}DEBUG{RESET}: I2C devices")
            for line in process.stdout:
                print(line, end="")
            
            process.wait()

            if process.returncode != 0:
                print(f"{BLUE}DEBUG{RESET}: I2C failed to detect with return code {process.returncode}")
        
        else:
            print(f"{BLUE}DEBUG{RESET}: I2C disabled. GPIO 2 and GPIO 3 are available as GPIO.\n")
        
        print(f"{BLUE}DEBUG{RESET}: I2C check completed. Proceeding to next command\n")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected Error - {e}")

# Check ToF sensor
def debug_tof(sensor):
    time.sleep(0.1)
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking VL53L1X status...")
        test_distance = tof.check_distance(sensor)
        if test_distance != -1:
            print(f"{BLUE}DEBUG{RESET}: VL53L1X in normal status - {test_distance}mm\n")
        else:
            print(f"{RED}ERROR{RESET}: VL53L1X not functioning - {test_distance}\n")
        print(f"{BLUE}DEBUG{RESET}: VL53L1X check completed. Proceeding to next command\n")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected Error - {e}")

# Check camera
def debug_picam():
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking PiCam status...")
        process = subprocess.Popen(['libcamera-hello', '--list-cameras'], 
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text = True)
        output = ""
        print(f"{BLUE}DEBUG{RESET}: Camera available:")
        for line in process.stdout:
            print(line, end="")
            output += line
        
        process.wait()

        if "No cameras available" in output:
            print(f"{RED}ERROR{RESET}: No cameras detected\n")
        elif "Available cameras" in output and len(output.splitlines()) > 1:
            print(f"{BLUE}DEBUG{RESET}: PiCam connected\n")
        else:
            print(f"{RED}ERROR{RESET}: Unexpected error\n")
        print(f"{BLUE}DEBUG{RESET}: PiCam check completed. Proceeding to next command\n")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected Error - {e}")

# Check serial
def debug_serial(ser, addr):
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking Serial status for <{addr}>...")
        process = subprocess.Popen(['ls', '-l', addr], 
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text = True)
        output = ""
        print(f"{BLUE}DEBUG{RESET}: Serial available:")
        for line in process.stdout:
            print(line, end="")
            output += line
        
        process.wait()
        
        if "dialout" in output and addr in output:
            print(f"{BLUE}DEBUG{RESET}: Dialout permission granted for <{addr}> UART connection\n")
        else:
            print(f"{RED}ERROR{RESET}: Permission denied for <{addr}>. Check UART permission\n")
        
        print(f"{BLUE}DEBUG{RESET}: Checking GPIO pins mode for <{addr}>")
        if addr.endswith("AMA3"):
            print(f"{BLUE}DEBUG{RESET}: Checking GPIO pins mode for UART_0")
            pins = "4,5"
        elif addr.endswith("AMA4"):
            pins = "8,9"
            print(f"{BLUE}DEBUG{RESET}: Checking GPIO pins mode for UART_4")
        else:
            print(f"{RED}ERROR{RESET}: Address {addr} mismatched\n")
            return

        process_2 = subprocess.Popen(['raspi-gpio', 'get', pins], 
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text = True)
        output_2 = ""
        print(f"{BLUE}DEBUG{RESET}: Mode and status for GPIO pins {pins}")
        for line in process_2.stdout:
            print(line, end="")
            output_2 += line
        
        process_2.wait()

        if addr.endswith("AMA0"):
            if "TXD3" in output_2 and "RXD3" in output_2:
                print(f"{BLUE}DEBUG{RESET}: GPIO pins {pins} ready for UART_3\n")
            else:
                print(f"{RED}ERROR{RESET}: GPIO pins {pins} not for UART purpose. Check Pi configurations\n")
        elif addr.endswith("AMA4"):
            if "TXD4" in output_2 and "RXD4" in output_2:
                print(f"{BLUE}DEBUG{RESET}: GPIO pins {pins} ready for UART_4\n")
            else:
                print(f"{RED}ERROR{RESET}: GPIO pins {pins} not for UART purpose. Check Pi configurations\n")

        print(f"{BLUE}DEBUG{RESET}: Serial check completed. Proceeding to next command\n")    
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")

# Check UART Sub Machine
def debug_uart(ser):
    connected = False
    count = 1
    max_retries = 5
    timeout = 3

    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking UART connection status for Sub Machine...")

        while not connected and count < max_retries:
            message = f"Ping {count} from Master Machine\n".encode('utf-8')
            response = False

            for retries in range (1,max_retries+1):
                ser.write(message)
                print(f"{BLUE}DEBUG{RESET}: Message sent to Sub Machine <{retries}>")

                start_time = time.time()
                while time.time() - start_time < timeout:
                    if ser.in_waiting > 0:
                        line = ser.readline()
                        print(line.hex())
                        feedback = line.decode('utf-8').strip()
                        print(f"{BLUE}DEBUG{RESET}: Received from Sub Machine")
                        print(feedback)
                        if feedback == "Message sent to Master Machine":
                            response = True
                            break
                
                    time.sleep(0.01)
                
                if response:
                    print(f"{BLUE}DEBUG{RESET}: Message received matched. UART connection available\n")
                    connected = True
                    break
                elif retries == max_retries:
                    print(f"{RED}ERROR{RESET}: Sub Machine takes too long to respond. Connection failed\n")
                    break
                else:
                    count += 1
                    print(f"{BLUE}DEBUG{RESET}: No feedback from Sub Machine or message received mismatched. Retry <{count}>\n")
                    continue
        
        print(f"{BLUE}DEBUG{RESET}: UART check for Sub Machine completed. Proceeding to next command\n") 
    
    except serial.SerialException as se:
        print(f"{RED}ERROR{RESET}: Unexpected serial error - {se}")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")

# Check UART for sc08
def debug_sc08(ser):
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking UART connection status for sc08...")

        for channel in range (1, 9):
            # Send
            print(f"{BLUE}DEBUG{RESET}: Checking channel {channel}")
            command = 0b10100000 | channel
            print(f"{BLUE}DEBUG{RESET}: Requesting angle from channel {channel}")
            ser.write(bytes([command]))
            time.sleep(0.5)

            # Receive
            high = ser.read(1)
            low = ser.read(1)

            if high and low:
                high_byte = high[0]
                low_byte = low[0]
                position = (high_byte << 6) | (low_byte & 0x3F)
                position = (position / 8000) * 180
                print(f"{BLUE}DEBUG{RESET}: Channel {channel} available - {position}\n")
                time.sleep(0.5)
            else:
                print(f"{RED}ERROR{RESET}: Channel {channel} not available. Check UART connection\n")
                time.sleep(0.5)
            
        print(f"{BLUE}DEBUG{RESET}: UART check for sc08 completed. Proceeding to next command\n")

    except serial.SerialException as se:
        print(f"{RED}ERROR{RESET}: Unexpected serial error - {se}")

    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")

# Check motor initial angle
def debug_initial_pos(ser, initial_pos):
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking initial position for motors...")

        for i in range (1, len(initial_pos)+1):
            channel = f"channel_{i}"
            print(f"{BLUE}DEBUG{RESET}: Checking motor {i} position at channel {channel}")

            # Send
            command = 0b10100000 | i
            ser.write(bytes([command]))
            time.sleep(0.5)

            # Receive 
            high = ser.read(1)
            low = ser.read(1)

            if not high is None and not low is None:
                high_byte = high[0]
                low_byte = low[0]
                position = (high_byte << 6) | (low_byte & 0x3F)
                position = (position / 8000) * 180
                print(f"{BLUE}DEBUG{RESET}: Angle information received from channel {i} - {position}")
                time.sleep(0.5)

                # Checking
                if (position >= initial_pos[channel] - 2) and (position <= initial_pos[channel] + 2):
                    print(f"{BLUE}DEBUG{RESET}: Motor {i} is at expected initial position\n")
                else:
                    print(f"{RED}ERROR{RESET}: Motor {i} is at unexpected position. Check motor {i} settings\n")
            else:
                print(f"{RED}ERROR{RESET}: No response from channel {i}. Check connection of motor {i}\n")
                time.sleep(0.5)

    except serial.SerialException as se:
        print(f"{RED}ERROR{RESET}: Unexpected serial error - {se}")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")