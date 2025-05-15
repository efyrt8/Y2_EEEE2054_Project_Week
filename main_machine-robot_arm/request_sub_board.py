import pigpio
import serial
import os, sys
import time

# Parse response (class)
def parse_response_class(response):
    # Format -> A:<a_num>,M:<m_num>,P:<p_num>
    try:
        parts = response.split(',')

        a_num = int(parts[0].split(':')[1])
        m_num = int(parts[1].split(':')[1])
        p_num = int(parts[2].split(':')[1])

        return a_num, m_num, p_num

    except Exception as e:
        return -1, -1, -1

# Parse response (counts)
def parse_response_count(response):
    # Format -> P:<pick-up-zone-obj>,W:<in_wait_obj>
    try:
        parts = response.split(',')

        pick_up_zone_obj = int(parts[0].split(':')[1])
        in_wait_obj = int(parts[1].split(':')[1])

        return pick_up_zone_obj, in_wait_obj
    
    except Exception as e:
        return -1, -1, -1

def parse_response_obj(response):
    # Format -> D:<pick-up-zone-obj>
    try:
        parts = response.split(':')

        pick_up_zone_obj = parts[1]

        return pick_up_zone_obj

    except Exception as e:
        return -1

# Request object on slider
def request_num(ser):
    ser.write(b"Send results\n")
    a_num, m_num, p_num = 0, 0, 0

    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline()
            response = line.decode('utf-8').strip()

            # Parse response
            a_num, m_num, p_num = parse_response_class(response)
            return a_num, m_num, p_num
        
        time.sleep(0.01)
    
    return -1, -1, -1

# Request total objects in wait
def request_count(ser):
    ser.write(b"Obj count\n")
    pick_up_zone_obj, in_wait_obj = 0, 0

    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline()
            response = line.decode('utf-8').strip()

            # Parse response
            pick_up_zone_obj, in_wait_obj = parse_response_count(response)
            return pick_up_zone_obj, in_wait_obj
        
        time.sleep(0.01)
    
    return -1, -1

# Deduct count of object in pick up zone
def deduct_count(ser):
    ser.write(b"Deduct count\n")
    pick_up_zone_obj = 0

    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline()
            response = line.decode('utf-8').strip()
            pick_up_zone_obj = parse_response_obj(response)

            return pick_up_zone_obj

        time.sleep(0.01)
    
    return -1

# Terminate process on Sub Machine
def end_process(ser):
    ser.write(b"Terminate process\n")

    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline()
            response = line.decode('utf-8').strip()

            if response == "end":
                return 1
            else:
                return -1
        
        time.sleep(0.01)
    
    return -2