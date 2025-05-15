# ONLY ENABLED WHEN DEBUG MODE IS SET TO ON
# TO SET DEBUG MODE TO ON, USE "-d on" WHEN RUNNING "main.py"

import pigpio
import time
import subprocess, sys, os
import serial

# Colour code
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
RESET="\033[0m"

pins_to_test = [6, 12, 13, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]

# Check pigpio daemon
def callback_test(pin, level, tick):
    print(f"{BLUE}DEBUG{RESET}: Edge detected in pin GPIO {pin}: {'HIGH' if level==1 else 'LOW'} at tick {tick}\n")

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

def debug_serial(ser, addr):
    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking Serial status for <{addr}>")
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

        if "TXD3" in output_2 and "RXD3" in output_2:
            print(f"{BLUE}DEBUG{RESET}: GPIO pins {pins} ready for UART_3\n")
        else:
            print(f"{RED}ERROR{RESET}: GPIO pins {pins} not for UART purpose. Check Pi configurations\n")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")

# Check UART
def debug_uart(ser):
    connected = False
    timeout = 30
    expired = False

    try:
        print(f"\n{BLUE}DEBUG{RESET}: Checking UART status...")

        start_time = time.time()
        while not connected and not expired:
            if ser.in_waiting > 0:
                line = ser.readline()
                receive = line.decode('utf-8').strip()
                print(receive)

                if "from Master Machine" in receive:
                    print(f"{BLUE}DEBUG{RESET}: Message received from Master Machine\n")
                    response = "Message sent to Master Machine\n".encode('utf-8')
                    # ser.write(b"Message sent to Master Machine\n")
                    ser.write(response)
                    print(f"{BLUE}DEBUG{RESET}: Confirmation message sent to Master Machine\n")
                    connected = True

            if time.time() - start_time >= timeout:
                print(f"{RED}ERROR{RESET}: Master Machine takes too long to respond. Connection failed\n")
                expired = True

            time.sleep(0.01)

        print(f"{BLUE}DEBUG{RESET}: UART check completed. Proceeding to next command\n") 
    
    except serial.SerialException as se:
        print(f"{RED}ERROR{RESET}: Unexpected serial error - {se}")
    
    except Exception as e:
        print(f"{RED}ERROR{RESET}: Unexpected error - {e}")