# Dual nozzle height calibration using an optical switch
#
# Copyright (C) 2023  Your Name <your@email.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import statistics

class DualNozzleHeightCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        
        try:
            self.switch_pin = config.get('switch_pin')
            ppins = self.printer.lookup_object('pins')
            pin_params = ppins.lookup_pin(self.switch_pin, can_invert=True, can_pullup=True)
            self.mcu_endstop = pin_params['chip'].setup_pin('endstop', pin_params)
        except Exception as e:
            raise config.error(f"Error initializing dual_nozzle_height_calibration: Unable to configure switch pin. {str(e)}")
        
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.probing_speed = config.getfloat('probing_speed', 1.0, above=0.)
        
        # 添加采样次数配置
        self.samples = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2., above=0.)
        
        # 设置Z轴步进电机
        self.z_steppers = []
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        
        self.gcode.register_command(
            'CALIBRATE_DUAL_NOZZLES', self.cmd_CALIBRATE_DUAL_NOZZLES,
            desc=self.cmd_CALIBRATE_DUAL_NOZZLES_help)
        
        self.logger = logging.getLogger(name="dual_nozzle_calibration")
        self.logger.setLevel(logging.DEBUG)

    def _handle_ready(self):
        toolhead = self.printer.lookup_object('toolhead')
        for stepper in toolhead.get_kinematics().get_steppers():
            if stepper.is_active_axis('z'):
                self.z_steppers.append(stepper)
        
        if not self.z_steppers:
            raise self.printer.config_error("No Z axis steppers found for dual nozzle height calibration")

    cmd_CALIBRATE_DUAL_NOZZLES_help = "Calibrate the height difference between two nozzles"
    def cmd_CALIBRATE_DUAL_NOZZLES(self, gcmd):
        try:
            self.logger.info("Starting dual nozzle calibration")
            self.gcode.run_script_from_command("BED_MESH_CLEAR")
            # Ensure the printer is homed
            toolhead = self.printer.lookup_object('toolhead')
            if 'xyz' not in toolhead.get_status(self.printer.get_reactor().monotonic())['homed_axes']:
                raise gcmd.error("Must home printer first")

            # Get values from Variables
            save_variables = self.printer.lookup_object('save_variables')
            status = save_variables.get_status(self.printer.get_reactor().monotonic())
            variables = status.get('variables', {})
            if not variables:
                self.logger.warning("No variables found in save_variables status")
            
            self.switch_position = [
                variables.get('switch_xpos', 0.),
                variables.get('switch_ypos', 0.),
                variables.get('zendstop', 0.)
            ]
            self.safe_z_height = variables.get('switch_zpos', 10.)
            
            self.logger.info(f"Using switch_position: {self.switch_position}")
            self.logger.info(f"Using safe_z_height: {self.safe_z_height}")

            # Measure height of first nozzle
            self.logger.info("Measuring height of first nozzle")
            self.gcode.run_script_from_command("T0")
            height1 = self._measure_nozzle_height(gcmd)
            self.logger.info(f"Height of first nozzle: {height1}")
            
            # Move to safe Z height before switching nozzles
            self._move_to_safe_z(gcmd)
            
            # Measure height of second nozzle
            self.logger.info("Measuring height of second nozzle")
            self.gcode.run_script_from_command("T1")
            height2 = self._measure_nozzle_height(gcmd, is_right_nozzle=True)
            self.logger.info(f"Height of second nozzle: {height2}")
            
            # Calculate height difference
            height_diff = round(height2 - height1, 3)  # 保留三位小数
            self.logger.info(f"Calculated height difference: {height_diff:.3f}")
            
            # Move to safe Z height
            self._move_to_safe_z(gcmd)
            
            # Save results to Variables
            save_gcmd = self.gcode.create_gcode_command('SAVE_VARIABLE', 'SAVE_VARIABLE', {'VARIABLE': 'e1_zoffset', 'VALUE': f"{height_diff:.3f}"})
            save_variables.cmd_SAVE_VARIABLE(save_gcmd)
            
            gcmd.respond_info(
                f"Nozzle height difference: {height_diff:.3f}\n"
                "The height difference has been saved to 'e1_zoffset' in Variables.\n"
                f"Dual nozzle height calibration completed. reboot to use the height difference: {height_diff:.3f}")
            self.logger.info(f"Dual nozzle calibration completed successfully. Height difference: {height_diff:.3f}")

            self.gcode.run_script_from_command("T0")  # 切换回左侧喷嘴

        except Exception as e:
            self.logger.error(f"Error during calibration: {str(e)}", exc_info=True)
            raise

    def _move_to_safe_z(self, gcmd):
        self.logger.info(f"Moving to safe Z height: {self.safe_z_height}")
        try:
            self.gcode.run_script_from_command(f"G1 Z{self.safe_z_height} F{self.lift_speed * 60}\nM400")
        except self.printer.command_error as e:
            self.logger.error(f"Error moving to safe Z height: {str(e)}")
            raise gcmd.error(str(e))

    def _measure_nozzle_height(self, gcmd, is_right_nozzle=False):
        self.logger.info(f"Starting nozzle height measurement for {'right' if is_right_nozzle else 'left'} nozzle")
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        self.logger.info(f"Current toolhead position: {toolhead.get_position()}")
        self.logger.info(f"Homed axes: {toolhead.get_status(curtime)['homed_axes']}")
        self.gcode.run_script_from_command("M400")
        
        # Move to safe Z height
        self._move_to_safe_z(gcmd)
        
        # Move to probing position XY
        probe_x = self.switch_position[0]
        probe_y = self.switch_position[1]

        
        self.logger.info(f"Moving to probing position XY: {probe_x}, {probe_y}")
        try:
            self.gcode.run_script_from_command(f"G1 X{probe_x} Y{probe_y} F{self.speed * 60}\nM400")
        except self.printer.command_error as e:
            self.logger.error(f"Error moving to probing position XY: {str(e)}")
            raise gcmd.error(str(e))
        
        # Probe
        self.logger.info(f"Starting probe with {self.samples} samples")
        measured_heights = []
        for i in range(self.samples):
            try:
                pos = self._probe(self.probing_speed)
                measured_height = pos[2]
                measured_heights.append(measured_height)
                self.logger.info(f"Sample {i+1}: Measured height: {measured_height}")
                
                # Retract between samples if not the last sample
                if i < self.samples - 1:
                    self._retract(pos)
            except self.printer.command_error as e:
                self.logger.error(f"Error during probing: {str(e)}")
                raise gcmd.error(str(e))
        
        # Calculate average height
        average_height = statistics.mean(measured_heights)
        self.logger.info(f"Average measured height: {average_height}")
        return average_height

    def _probe(self, speed):
        self.logger.info(f"Probing at speed: {speed}")
        toolhead = self.printer.lookup_object('toolhead')
        phoming = self.printer.lookup_object('homing')
        curpos = toolhead.get_position()
        self.logger.info(f"Current position before probe: {curpos}")
        pos = curpos[:]
        pos[2] = self.switch_position[2]
        self.logger.info(f"Target probe position: {pos}")
        
        self.logger.info(f"Z axis steppers: {self.z_steppers}")
        
        # 创建一个新的 MCU_Endstop 对象，包含 Z 轴步进电机
        for stepper in self.z_steppers:
            self.mcu_endstop.add_stepper(stepper)
        
        try:
            epos = phoming.probing_move(self.mcu_endstop, pos, speed)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += "\nMake sure the optical switch is connected and functioning correctly."
            self.logger.error(f"Error during probing move: {reason}")
            raise self.printer.command_error(reason)
        self.logger.info(f"Probe successful, position: {epos}")
        return epos

    def _retract(self, pos):
        self.logger.info(f"Retracting by {self.sample_retract_dist}mm")
        target_z = pos[2] + self.sample_retract_dist
        try:
            self.gcode.run_script_from_command(f"G1 Z{target_z} F{self.lift_speed * 60}\nM400")
        except self.printer.command_error as e:
            self.logger.error(f"Error during retraction: {str(e)}")
            raise

def load_config(config):
    return DualNozzleHeightCalibration(config)
