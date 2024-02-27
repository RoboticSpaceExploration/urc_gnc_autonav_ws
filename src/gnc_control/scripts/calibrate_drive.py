#!/usr/bin/python3
import odrive
from odrive.utils import *
from odrive.enums import *
import rospkg
import json

def main():
    print("How to use:")
    print("    Enter the numbers for which motors to calibrate")
    print("    Example: entering '01' will calibrate the front and")
    print("    back left motors. Entering '0123' will calibrate all motors")
    print("Calibration options: ")
    print("    0 - Motor controller 0, axis 0 (Front left)")
    print("    1 - Motor controller 0, axis 0 (Back left)")
    print("    2 - Motor controller 0, axis 0 (Front right)")
    print("    3 - Motor controller 0, axis 0 (Back right)")
    option = input("Enter options: ")

    rospack = rospkg.RosPack()
    json_path = rospack.get_path('gnc_control') + '/config/odrive_7pole.json'

    root = None
    with open(json_path) as file:
        json_text = file.read()
        decoder = json.decoder.JSONDecoder()
        root = decoder.decode(json_text)
        # format of json is a list of option objects
        # with properties: name, type, value
    odrv0 = odrive.find_any(serial_number='2052325E4D31')
    print(dir(odrv0))
    print(rate_test(odrv0))
    
    # if '0' in option or '1' in option:
    #     odrv0 = odrive.find_any(serial_number='2052325E4D31')
        
    #     if '0' in option:
    #         print('Calibrating axis0 motor')
    #         odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    #         print(odrv0.axis0.current_state)
    #         print('Saving configuration')
    #     if '1' in option:
    #         odrv0.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    #         print(odrv0.axis1.current_state)



if __name__ == "__main__":
    main()