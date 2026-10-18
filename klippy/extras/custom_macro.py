# Support for 1-wire based temperature sensors
#
# Copyright (C) 2020 Alan Lord <alanslists@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import time

class CUSTOM_MACRO:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.extruder_temp=None
        self.bed_temp=None
        self.gcode.register_command("CX_PRINT_LEVELING_CALIBRATION", self.cmd_CX_PRINT_LEVELING_CALIBRATION, desc=self.cmd_CX_PRINT_LEVELING_CALIBRATION_help)
        self.gcode.register_command("CX_PRINT_DRAW_ONE_LINE", self.cmd_CX_PRINT_DRAW_ONE_LINE, desc=self.cmd_CX_PRINT_DRAW_ONE_LINE_help)
        self.gcode.register_command("CX_ROUGH_G28", self.cmd_CX_ROUGH_G28, desc=self.cmd_CX_ROUGH_G28_help)
        self.gcode.register_command("CX_NOZZLE_CLEAR", self.cmd_CX_NOZZLE_CLEAR, desc=self.cmd_CX_NOZZLE_CLEAR_help)
        self.default_extruder_temp = config.getfloat("default_extruder_temp", default=240.0)
        self.default_bed_temp = config.getfloat("default_bed_temp", default=50.0)
        self.g28_ext_temp = config.getfloat("g28_ext_temp", default=140.0)
        self.leveling_calibration = 0
        pass


    def get_status(self, eventtime):
        return {
            'leveling_calibration': self.leveling_calibration,
            'default_extruder_temp': self.default_extruder_temp,
            'default_bed_temp': self.default_bed_temp,
            'g28_ext_temp': self.g28_ext_temp,
        }

    cmd_CX_PRINT_LEVELING_CALIBRATION_help = "Start Print function,three parameter:EXTRUDER_TEMP(180-300),BED_TEMP(0-100),CALIBRATION(0 or 1)"
    def cmd_CX_PRINT_LEVELING_CALIBRATION(self, gcmd):
        self.gcode.run_script_from_command('CHECK_BED_MESH AUTO_G29=1')
        pass

    cmd_CX_PRINT_DRAW_ONE_LINE_help = "Draw one line before printing"
    def cmd_CX_PRINT_DRAW_ONE_LINE(self, gcmd):
        self.gcode.run_script_from_command('G28 X Y') # Home the X and Y axes
        self.gcode.run_script_from_command('M83') # Enable relative extrusion
        self.gcode.run_script_from_command('G1 X10 Y10 Z2 F6000') # Move to X=10, Y=10, Z=2 at 6000 mm/min (100 mm/s)
        self.gcode.run_script_from_command('G1 Z0.1 F600') # Move Z to 0.1 at 600 mm/min (10 mm/s)
        pheaters = self.printer.lookup_object('heaters')
        heater_hot = self.printer.lookup_object('extruder').heater # Retrieve the hotend heater
        self.gcode.run_script_from_command('M104 S%d' % (self.extruder_temp)) # Set hotend temperature
        self.gcode.run_script_from_command('M140 S%d' % (self.bed_temp)) # Set bed temperature
        pheaters.set_temperature(heater_hot, self.extruder_temp, True) # set hotent temperature and wait until temperature is reached
        self.gcode.run_script_from_command('G21') # Set units to millimeters
        self.gcode.run_script_from_command('G1 F2400 E-0.5') # Retract filament by 0.5 mm at a feed rate of 2400 mm/min (40 mm/s)
        self.gcode.run_script_from_command('SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5')
        self.gcode.run_script_from_command('M204 S12000') # Request an acceleration setting of 12000 mm/s²
        self.gcode.run_script_from_command('G21') # Set units to millimeters, redundant
        self.gcode.run_script_from_command('SET_VELOCITY_LIMIT ACCEL_TO_DECEL=6000')
        self.gcode.run_script_from_command('M220 S100') # Set the print speed factor to 100% (normal speed)
        self.gcode.run_script_from_command('M221 S100') # Sets the extrusion flow factor to 100% (normal flow)
        self.gcode.run_script_from_command('G1 Z2.0 F1200') # Move Z to 2.0 at 1200 mm/min (20 mm/s)
        self.gcode.run_script_from_command('G1 X0.1 Y20 Z0.3 F6000.0') # Move to X=0.1, Y=20, Z=0.3 at 6000 mm/min (100 mm/s)
        self.gcode.run_script_from_command('G1 X0.1 Y180.0 Z0.3 F3000.0 E10.0') # Move along the Y-axis to Y=180 at 3000 mm/min (50 mm/s) and extrude 10 mm
        self.gcode.run_script_from_command('G1 X0.4 Y180.0 Z0.3 F3000.0') # Move sideways to X=0.4, Y=180, Z=0.3 at 3000 mm/min (50 mm/s)
        self.gcode.run_script_from_command('G1 X0.4 Y20.0 Z0.3 F3000.0 E10.0') # Move back along the Y-axis to Y=20 at 3000 mm/min (50 mm/s) and extrude 10 mm
        self.gcode.run_script_from_command('G1 Y10.0 F3000.0') # Moves along Y to Y=10.0 at 3000 mm/min (50 mm/s)
        self.gcode.run_script_from_command('G1 Z2.0 F600.0') # Raise the nozzle to Z=2.0 at 600 mm/min (10 mm/s)
        self.gcode.run_script_from_command('G1 Z0.3 F600.0') # Lower the nozzle to Z=0.3 at 600 mm/min (10 mm/s)
        self.gcode.run_script_from_command('G1 Z2.0 F600.0') # Raise the nozzle to Z=2.0 at 600 mm/min (10 mm/s)
        self.gcode.run_script_from_command('M82') # Enable absolute extrusion
        self.gcode.run_script_from_command('G92 E0') # Reset the current extrusion coordinate to zero without moving the extruder
        self.gcode.run_script_from_command('G1 F12000') # Set the speed to 12000 mm/min (200 mm/s) for subsequent moves
        self.gcode.run_script_from_command('G21') # Set units to millimeters, redundant
        pass

    cmd_CX_ROUGH_G28_help = "rough G28"
    def cmd_CX_ROUGH_G28(self, gcmd):
        self.extruder_temp = gcmd.get_float('EXTRUDER_TEMP', default=self.default_extruder_temp, minval=180.0, maxval=320.0)
        self.g28_ext_temp = self.extruder_temp - 90
        if self.g28_ext_temp > 180.0:
            self.g28_ext_temp = 180.0
        try:
            prtouch = self.printer.lookup_object('prtouch_v2')
        except:
            prtouch = self.printer.lookup_object('prtouch')
            gcmd.respond_info("prtouch")
        prtouch.change_hot_min_temp(self.g28_ext_temp)
        self.bed_temp = gcmd.get_float('BED_TEMP', default=self.default_bed_temp, minval=0.0, maxval=130.0)
        self.leveling_calibration = gcmd.get_int('LEVELING_CALIBRATION', default=1, minval=0, maxval=1)
        self.gcode.run_script_from_command('M104 S%d' % (self.g28_ext_temp)) # Set hotend temperature
        self.gcode.run_script_from_command('M140 S%d' % (self.bed_temp)) # Set bed temperature
        self.gcode.run_script_from_command('M204 S500') # Request an acceleration setting of 500 mm/s²
        self.gcode.run_script_from_command('G28') # Home all axes
        pass

    cmd_CX_NOZZLE_CLEAR_help = "nozzle clear with temperature"
    def cmd_CX_NOZZLE_CLEAR(self, gcmd):
        self.gcode.run_script_from_command('NOZZLE_CLEAR HOT_MIN_TEMP=%d HOT_MAX_TEMP=%d BED_MAX_TEMP=%d' % (self.g28_ext_temp, self.extruder_temp - 20, self.bed_temp))
        pass

def load_config(config):
    return CUSTOM_MACRO(config)
