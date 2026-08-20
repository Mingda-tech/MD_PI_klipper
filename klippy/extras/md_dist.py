# This is used to control the MINGDA distance sensor

import threading
import logging
import chelper
import pins
import math
import time
import queue
import json
import struct
import numpy as np
import copy
from numpy.polynomial import Polynomial
from . import manual_probe
from . import probe
from . import bed_mesh
from . import thermistor
from . import adc_temperature
from mcu import MCU, MCU_trsync
from clocksync import SecondarySync

STREAM_BUFFER_LIMIT_DEFAULT = 100
STREAM_TIMEOUT = 5.0
probe_object_name = "probe"

class MD_Dist_Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()

        self.version = config.getfloat(
            "version", 1.0, minval=0.0
        )
        self.speed = config.getfloat(
            "speed", 5.0, above=0.0
        )
        self.lift_speed = config.getfloat(
            "lift_speed", self.speed, above=0.0
        )
        self.backlash_comp = config.getfloat(
            "backlash_comp", 0.5
        )
        self.x_offset = config.getfloat(
            "x_offset", 0.0
        )
        self.y_offset = config.getfloat(
            "y_offset", 0.0
        )
        self.trigger_distance = config.getfloat(
            "trigger_distance", 2.0
        )
        self.trigger_dive_threshold = config.getfloat(
            "trigger_dive_threshold", 1.0
        )
        self.trigger_hysteresis = config.getfloat(
            "trigger_hysteresis", 0.006
        )
        self.z_settling_time = config.getint(
            "z_settling_time", 5, minval=0
        )

        # If using paper for calibration, this would be .1mm
        self.cal_nozzle_z = config.getfloat(
            "cal_nozzle_z", 0.1
        )
        self.cal_floor = config.getfloat(
            "cal_floor", 0.2
        )
        self.cal_ceil = config.getfloat(
            "cal_ceil", 5.0
        )
        self.cal_speed = config.getfloat(
            "cal_speed", 1.0
        )
        self.cal_move_speed = config.getfloat(
            "cal_move_speed", 10.0
        )

        self.end_park_x, self.end_park_y = config.getfloatlist(
            "end_park_pos", [None, None], count=2
        )
        self.first_move_axis = config.getchoice(
            "first_move_axis", {'x':'x', 'X':'x', 'y':'y', 'Y':'y'}, 'y'
        )
        self.enable_temp_compensate = config.getboolean(
            "enable_temp_compensate", True
        )
        self.use_known_temp_sensor = config.getboolean(
            "use_known_temp_sensor", False
        )
        self.z_virtual_endstop = config.getboolean(
            "z_virtual_endstop", True
        )
        self.default_samples = self.trigger_threshold = 0
        self.z_position = 0.
        self.samples_mode = 'normal'
        # TODO: Version 3.0 specific settings
        self.version_int = int(self.version*100)
        if self.version_int == 300:
            # get params from config
            self.trigger_threshold = config.getint(
                "trigger_threshold", minval=100)
            self.z_position = config.getfloat('probe_min', 0.)
            # Multi-sample support (for improved accuracy)
            sample_count = config.getint('samples', 1, minval=1)
            self.default_samples = sample_count
            self.sample_retract_dist = config.getfloat(
                'sample_retract_dist', 2., above=0.)
            atypes = ['median', 'average']
            mode = ['decreasing', 'normal', 'plummet']
            self.samples_result = config.getchoice(
                'samples_result', atypes, 'average')
            self.samples_tolerance = config.getfloat(
                'samples_tolerance', 0.100, minval=0.)
            self.samples_retries = config.getint(
                'samples_tolerance_retries', 0, minval=0)
            # new mode
            self.samples_mode = config.getchoice(
                'samples_mode', mode, 'normal')
            self.decr_de = config.getfloat(
                'decreasing_dist_end', default=.5, minval=.5)
            self.decr_ds = config.getfloat(
                'decreasing_dist_start',
                default=self.sample_retract_dist,
                minval=self.decr_de)
            self.decr_se = config.getfloat(
                'decreasing_speed_end', default=1., minval=.1)
            self.decr_ss = config.getfloat(
                'decreasing_speed_start', default=self.speed,
                minval=self.decr_se)
            self.plummet_num = config.getint(
                'plummet_num', int(sample_count/2),
                1, (sample_count-1))
            self.plummet_speed = config.getfloat(
                'plummet_speed', self.speed/2, 0.1, self.speed)
            # Infer Z position to move to during a probe
            if config.has_section('stepper_z'):
                zconfig = config.getsection('stepper_z')
                self.z_position = zconfig.getfloat('position_min', 0.,
                                                note_valid=False)
            else:
                pconfig = config.getsection('printer')
                self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                                note_valid=False)
            # self.trigger_distance = 0.0

        # Load models
        self.model = None
        self.models = {}
        self.temp_t1 = config.getfloat(
            "temperature1", 25.0, minval=thermistor.KELVIN_TO_CELSIUS)
        self.temp_r1 = config.getfloat(
            "resistance1", 47000.0, minval=0.)
        self.temp_beta = config.getfloat(
            "beta", 4041.0, above=0.)
        self.temp_pullup = config.getfloat(
            'pullup_resistor', 10000.0, above=0.)
        self.temp_inline_resistor = config.getfloat(
            'inline_resistor', 0., minval=0.)
        self.model_temp_builder = MD_Dist_Temp_Model_Builder.load(config)
        self.model_temp = None
        self.fmin = None
        self.default_model_name = config.get(
            "default_model_name",
            "default"
        )
        self.model_manager = Model_Manager(self)

        # Temperature sensor integration
        self.last_temp = 0
        self.measured_min = 99999999.0
        self.measured_max = 0.0

        self.last_sample = None
        self.hardware_failure = None

        self.mesh_helper = MD_Dist_Mesh_Helper.create(self, config)

        self._stream_en = 0
        self._stream_timeout_timer = self.reactor.register_timer(
            self._stream_timeout
        )
        self._stream_callbacks = {}
        self._stream_latency_requests = {}
        self._stream_buffer = []
        self._stream_buffer_limit = STREAM_BUFFER_LIMIT_DEFAULT
        self._stream_buffer_limit_new = self._stream_buffer_limit
        self._stream_samples_queue = queue.Queue()
        self._stream_flush_event = threading.Event()
        self._log_stream = None
        self._data_filter = Alpha_Beta_Filter(
            config.getfloat("filter_alpha", 0.5),
            config.getfloat("filter_beta", 0.000001),
        )
        self.trapq = None

        self.sensor_freq = config.getint('sensor_freq', None)
        # Chip options
        dout_pin_name = config.get('dout_pin', None)
        sclk_pin_name = config.get('sclk_pin', None)
        if (dout_pin_name is not None) and (sclk_pin_name is not None):
            soft_i2c = config.getint('use_software_i2c', 0)
            ppins = self.printer.lookup_object('pins')
            self.addr = config.getint('addr', 0x2A)
            if self.version_int == 100:
                temp_pin_name = config.get('temp_pin')
                temp_ppin = ppins.lookup_pin(temp_pin_name)
                dout_ppin = ppins.lookup_pin(dout_pin_name)
                sclk_ppin = ppins.lookup_pin(sclk_pin_name)
                self._mcu = mcu = dout_ppin['chip']
                # self.oid = mcu.create_oid()
                if ((sclk_ppin['chip'] is not mcu) or
                    (temp_ppin['chip'] is not mcu)):
                    raise config.error("%s config error: All pins must be "
                                    "connected to the same MCU" % (self.name,))
                dout_pin = dout_ppin['pin']
                sclk_pin = sclk_ppin['pin']
                temp_pin = temp_ppin['pin']
                logging.info(
                    "md_dist_config dout_pin=%s sclk_pin=%s temp_pin=%s"
                    " addr=%s soft_i2c=%s"
                    % (dout_pin, sclk_pin, temp_pin, self.addr, soft_i2c,))
                mcu.add_config_cmd(
                    "md_dist_config dout_pin=%s sclk_pin=%s temp_pin=%s"
                    " addr=%s soft_i2c=%s"
                    % (dout_pin, sclk_pin, temp_pin, self.addr, soft_i2c,))
            elif (self.version_int == 200) or (self.version_int == 300):
                # get pin
                temp_pin_name = config.get('temp_pin', None)
                if temp_pin_name is not None:
                    temp_ppin = ppins.lookup_pin(temp_pin_name)
                elif not self.use_known_temp_sensor:
                    self.enable_temp_compensate = False
                dout_ppin = ppins.lookup_pin(dout_pin_name)
                sclk_ppin = ppins.lookup_pin(sclk_pin_name)
                self._mcu = mcu = dout_ppin['chip']
                # self.oid = mcu.create_oid()
                if ((sclk_ppin['chip'] is not mcu) or
                    (temp_pin_name is not None and
                    temp_ppin['chip'] is not mcu)):
                    raise config.error("%s config error: All pins must be "
                                    "connected to the same MCU" % (self.name,))
                dout_pin = dout_ppin['pin']
                sclk_pin = sclk_ppin['pin']
                if temp_pin_name is not None:
                    temp_pin = temp_ppin['pin']
                    mcu.add_config_cmd(
                        "md_dist_temp_config temp_pin=%s" % (temp_pin,))
                    logging.info("md_dist has temp_pin=%s" % (temp_pin,))

                # get config data
                external_clock = config.getint('external_clock', 0)
                clock_div = config.getint(
                    'clock_div', 1, minval=1, maxval=1023)
                rcount_clock = config.getint(
                    'rcount_clock', 128, minval=128, maxval=65535)
                settle_clock = config.getint(
                    'settle_clock', 2, minval=2, maxval=65535)
                drive_current = config.getint(
                    'drive_current', 1, minval=1, maxval=31)
                logging.info(
                    "md_dist_config_data external_clock=%s clock_div=%s"
                    " rcount=%s settle=%s current=%s"
                    % (external_clock, clock_div,
                       rcount_clock, settle_clock, drive_current,))
                mcu.add_config_cmd(
                    "md_dist_config_data external_clock=%s clock_div=%s"
                    " rcount=%s settle=%s current=%s"
                    % (external_clock, clock_div,
                       rcount_clock, settle_clock, drive_current,))

                logging.info(
                    "md_dist_config dout_pin=%s sclk_pin=%s addr=%s"
                    " soft_i2c=%s"
                    % (dout_pin, sclk_pin, self.addr, soft_i2c,))
                mcu.add_config_cmd(
                    "md_dist_config dout_pin=%s sclk_pin=%s addr=%s"
                    " soft_i2c=%s"
                    % (dout_pin, sclk_pin, self.addr, soft_i2c,))
        else:
            mainsync = self.printer.lookup_object("mcu")._clocksync
            self._mcu = MCU(config, SecondarySync(self.reactor, mainsync))
            self.printer.add_object("mcu " + self.name, self._mcu)
        self.cmd_queue = self._mcu.alloc_command_queue()
        self.mcu_probe = MD_Dist_Endstop_Wrapper(self)

        if not self.enable_temp_compensate:
            self.use_known_temp_sensor = False
        if self.use_known_temp_sensor:
            self.sensor_id = config.get('sensor_id', 'B')
        self.sensor_obj = None

        global probe_object_name
        probe_object_name = config.get('probe_object', "probe")
        # Register z_virtual_endstop
        if self.z_virtual_endstop:
            self.printer.lookup_object("pins").register_chip(
                probe_object_name, self
            )
        # Register event handlers
        self.printer.register_event_handler(
            "klippy:connect",
            self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:mcu_identify",
            self._handle_mcu_identify
        )
        self._mcu.register_config_callback(self._build_config)
        self._mcu.register_response(
            self._handle_md_dist_data,
            "md_dist_data"
        )
        # Register webhooks
        webhooks = self.printer.lookup_object("webhooks")
        self._api_dump_helper = API_Dump_Helper(self)
        webhooks.register_endpoint("md_dist/status", self._handle_req_status)
        webhooks.register_endpoint("md_dist/dump", self._handle_req_dump)

        # Register gcode commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "MD_DIST_QUERY",
            self.cmd_MD_DIST_QUERY,
            desc=self.cmd_MD_DIST_QUERY_help
        )
        if self.version_int != 300:
            self.gcode.register_command(
                "MD_DIST_STREAM",
                self.cmd_MD_DIST_STREAM,
                desc=self.cmd_MD_DIST_STREAM_help
            )
            self.gcode.register_command(
                "MD_DIST_CALIBRATE",
                self.cmd_MD_DIST_CALIBRATE,
                desc=self.cmd_MD_DIST_CALIBRATE_help
            )
            self.gcode.register_command(
                "MD_DIST_ESTIMATE_BACKLASH",
                self.cmd_MD_DIST_ESTIMATE_BACKLASH,
                desc=self.cmd_MD_DIST_ESTIMATE_BACKLASH_help
            )
        if self.z_virtual_endstop:
            self.gcode.register_command(
                "PROBE",
                self.cmd_PROBE,
                desc=self.cmd_PROBE_help
            )
            self.gcode.register_command(
                "PROBE_ACCURACY",
                self.cmd_PROBE_ACCURACY,
                desc=self.cmd_PROBE_ACCURACY_help
            )
            self.gcode.register_command(
                "Z_OFFSET_APPLY_PROBE",
                self.cmd_Z_OFFSET_APPLY_PROBE,
                desc=self.cmd_Z_OFFSET_APPLY_PROBE_help
            )
        else:
            # If z_virtual_endstop is not used,
            # the z_offset_apply of md_dist needs to be preserved
            self.gcode.register_command(
                "Z_OFFSET_APPLY_MD_DIST",
                self.cmd_Z_OFFSET_APPLY_PROBE,
                desc=self.cmd_Z_OFFSET_APPLY_PROBE_help
            )

    # Event handlers
    def _handle_connect(self):
        self.phoming = self.printer.lookup_object("homing")

        # Ensure streaming mode is stopped
        self.md_dist_stream_cmd.send([0])

        self.model_temp = self.model_temp_builder.build_with_base(self)
        if self.model_temp:
            self.fmin = self.model_temp.fmin
        self.model = self.models.get(self.default_model_name, None)
        if (self.model) and (self.version_int != 300):
            self._apply_threshold()
        if self.use_known_temp_sensor:
            self.pheaters = self.printer.lookup_object('heaters')
            self.sensor_obj = self._get_sensor_object(self.sensor_id)

    def _get_sensor_object(self, sensor_id):
        if sensor_id in self.pheaters.gcode_id_to_sensor:
            return self.pheaters.gcode_id_to_sensor[sensor_id]
        else:
            raise self.printer.config_error(
                "Unknown sensor ID: %s" % sensor_id)

    def _handle_mcu_identify(self):
        constants = self._mcu.get_constants()

        if self.sensor_freq is None:
            if self._mcu._mcu_freq < 20000000:
                self.sensor_freq =  self._mcu._mcu_freq
            elif self._mcu._mcu_freq < 100000000:
                self.sensor_freq = self._mcu._mcu_freq/2
            else:
                self.sensor_freq = self._mcu._mcu_freq/6

        self.inv_adc_max = 1.0 / constants.get("ADC_MAX")
        self.temp_smooth_count = constants.get("MD_DIST_ADC_SMOOTH_COUNT")
        self.thermistor = thermistor.Thermistor(
            self.temp_pullup, self.temp_inline_resistor)
        self.thermistor.setup_coefficients_beta(
            self.temp_t1, self.temp_r1, self.temp_beta)

        self.toolhead = self.printer.lookup_object("toolhead")
        self.trapq = self.toolhead.get_trapq()

    def _build_config(self):
        self.md_dist_stream_cmd = self._mcu.lookup_command(
            "md_dist_stream en=%u",
            cq=self.cmd_queue
        )
        self.md_dist_set_threshold_cmd = self._mcu.lookup_command(
            "md_dist_set_threshold trigger=%u untrigger=%u",
            cq=self.cmd_queue
        )
        self.md_dist_home_cmd = self._mcu.lookup_command(
            "md_dist_home trsync_oid=%c trigger_reason=%c trigger_invert=%c",
            cq=self.cmd_queue
        )
        self.md_dist_stop_home_cmd = self._mcu.lookup_command(
            "md_dist_stop_home",
            cq=self.cmd_queue
        )
        self.md_dist_base_read_cmd = self._mcu.lookup_query_command(
            "md_dist_base_read len=%c offset=%hu",
            "md_dist_base_data bytes=%*s offset=%hu",
            cq=self.cmd_queue
        )

    def stats(self, eventtime):
        return False, "%s: coil_temp=%.1f" % (self.name, self.last_temp)

    # Virtual endstop
    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop" or pin_params["pin"] != "z_virtual_endstop":
            raise pins.error(
                "Probe virtual endstop only useful as endstop pin"
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error(
                "Can not pullup/invert probe virtual endstop"
            )
        return self.mcu_probe

    # Probe interface
    def multi_probe_begin(self):
        logging.info("M_D: multi_probe_begin2")
        self._start_streaming()

    def multi_probe_end(self):
        logging.info("M_D: multi_probe_end2")
        self._stop_streaming()

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.trigger_distance

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        return self.lift_speed

    def get_probe_params(self, gcmd=None):
        # TODO: 获取探针参数
        if gcmd is None:
            gcmd = self.dummy_gcode_cmd
        probe_speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        samples = gcmd.get_int("SAMPLES", self.default_samples, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)

        samples_mode = 'normal'
        plum_n = 0
        plum_s = decr_ds = decr_de = decr_ss = decr_se = 0.
        if samples > 1:
            samples_mode = gcmd.get("SAMPLES_MODE", self.samples_mode)
            if (samples_mode == 'decreasing'):
                decr_de = gcmd.get_float("DECR_DIST_E", self.decr_de)
                decr_ds = gcmd.get_float("DECR_DIST_S", self.decr_ds,
                                        minval=decr_de)
                decr_se = gcmd.get_float("DECR_SPEED_E", self.decr_se)
                decr_ss = gcmd.get_float("DECR_SPEED_S", self.decr_ss,
                                        minval=decr_se)
            elif (samples_mode == 'plummet'):
                plum_n = gcmd.get_int("PLUMMET_NUM", self.plummet_num, 1,
                                      samples)
                plum_s = gcmd.get_float("PLUMMET_SPEED", self.plummet_speed)

        return {'probe_speed': probe_speed,
                'lift_speed': lift_speed,
                'samples': samples,
                'sample_retract_dist': sample_retract_dist,
                'samples_tolerance': samples_tolerance,
                'samples_tolerance_retries': samples_retries,
                'samples_result': samples_result,
                'samples_mode': samples_mode,
                'decr_dist_s': decr_ds,
                'decr_dist_e': decr_de,
                'decr_speed_s': decr_ss,
                'decr_speed_e': decr_se,
                'plummet_num': plum_n,
                'plummet_speed': plum_s,
            }

    def run_probe(self, gcmd):
        if (self.model is None) and (self.version_int != 300):
            raise self.printer.command_error("No md_dist model loaded")

        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        allow_faulty = gcmd.get_int("ALLOW_FAULTY_COORDINATE", 0) != 0
        lift_speed = self.get_lift_speed(gcmd)
        # toolhead = self.printer.lookup_object("toolhead")
        curtime = self.reactor.monotonic()
        if 'xyz' not in self.toolhead.get_status(curtime)["homed_axes"]:
            raise self.printer.command_error("Must home before probe")

        self._start_streaming()
        try:
            if self.version_int == 300:
                # get params
                params = self.get_probe_params(gcmd)
                probexy = self.toolhead.get_position()[:2]
                retries = 0
                positions = []
                sample_count = params['samples']
                if (params['samples_mode'] == 'decreasing'):
                    if sample_count > 1:
                        decr_dist_v = (
                            (params['decr_dist_s']-params['decr_dist_e'])
                            / (sample_count-1)) # (max-min)/num
                        decr_speed_v = int(
                            (params['decr_speed_s']-params['decr_speed_e'])
                            / (sample_count-1)) # (max-min)/num
                        decr_dist_list = [
                            (params['decr_dist_s']-(decr_dist_v*i))
                            for i in range(sample_count)]
                        decr_speed_list = [
                            (params['decr_speed_s']-(decr_speed_v*i))
                            for i in range(sample_count)]
                    else:
                        decr_dist_list = [params['decr_dist_s']]
                        decr_speed_list = [params['decr_speed_s']]
                    current_count = 0
                    last_z = self.z_position
                    while current_count < sample_count:
                        # Probe position
                        self.toolhead.wait_moves()
                        end_pos = self.toolhead.get_position()
                        end_pos[2] = last_z
                        # retract_dist = decr_dist_list[current_count]
                        cur_speed = decr_speed_list[current_count]
                        pos = self._probe_2(cur_speed, end_pos, current_count)
                        positions.append(pos)
                        current_count += 1
                        # Check samples tolerance
                        if ((current_count>1) and
                            (abs(last_z-pos[2])>(2*params['samples_tolerance']))
                            ):
                            if retries >= params['samples_tolerance_retries']:
                                raise gcmd.error(
                                    "Probe samples exceed samples_tolerance")
                            gcmd.respond_info(
                                "Probe samples exceed tolerance. Retrying...")
                            retries += 1
                            positions = []
                            current_count = 0
                            last_z = self.z_position
                        else:
                            last_z = (pos[2] - 
                                      max(0.3, params['samples_tolerance']))
                        # Retract
                        if current_count < sample_count:
                            self.toolhead.manual_move(
                                probexy + [decr_dist_list[current_count]],
                                decr_speed_list[current_count])
                    # Calculate result
                    # epos = probe.calc_probe_z_average(
                    #            positions, params['samples_result'])
                    epos = positions[-1]
                    logging.info("run_probe:%s,%s,%s" %
                        (decr_dist_list, decr_speed_list, epos,))
                elif (params['samples_mode'] == 'plummet'):
                    current_count = 0
                    tns = int(sample_count - params['plummet_num'])
                    last_z = self.z_position
                    while current_count < sample_count:
                        self.toolhead.wait_moves()
                        end_pos = self.toolhead.get_position()
                        end_pos[2] = last_z
                        if current_count < tns:
                            if retries:
                                cur_speed = params['plummet_speed']
                            else:
                                cur_speed = params['probe_speed']
                            pos = self._probe_2(
                                cur_speed, end_pos, current_count)
                        else:
                            cur_speed = params['plummet_speed']
                            pos = self._probe_2(
                                cur_speed, end_pos, current_count)
                            positions.append(pos)
                            last_z = (pos[2] -
                                      max(0.3, params['samples_tolerance']))
                            # Check samples tolerance
                            z_positions = [p[2] for p in positions]
                            tolerance = max(z_positions) - min(z_positions)
                            if tolerance > params['samples_tolerance']:
                                if retries>=params['samples_tolerance_retries']:
                                    raise gcmd.error(
                                        "Probe samples exceed samples_tolerance"
                                        )
                                gcmd.respond_info(
                                    "Probe samples exceed tolerance."
                                    " Retrying... %.4f"
                                    % (pos[2],))
                                retries += 1
                                positions = []
                                current_count = 0
                                last_z = self.z_position
                                self.toolhead.manual_move(probexy + [pos[2]+3.0],
                                                     cur_speed)
                                continue
                        current_count += 1
                        # Retract
                        if current_count < sample_count:
                            self.toolhead.manual_move(
                                (probexy +
                                 [pos[2] + params['sample_retract_dist']]),
                                cur_speed)
                    # Calculate result
                    epos = probe.calc_probe_z_average(positions,
                                                      params['samples_result'])
                else:
                    while len(positions) < sample_count:
                        # Probe position
                        self.toolhead.wait_moves()
                        end_pos = self.toolhead.get_position()
                        end_pos[2] = self.z_position
                        pos = self._probe_2(params['probe_speed'], end_pos)
                        positions.append(pos)
                        # Check samples tolerance
                        z_positions = [p[2] for p in positions]
                        tolerance = max(z_positions)-min(z_positions)
                        if tolerance > params['samples_tolerance']:
                            if retries >= params['samples_tolerance_retries']:
                                raise gcmd.error(
                                    "Probe samples exceed samples_tolerance")
                            gcmd.respond_info(
                                "Probe samples exceed tolerance. Retrying...")
                            retries += 1
                            positions = []
                        # Retract
                        if len(positions) < sample_count:
                            self.toolhead.manual_move(
                                (probexy +
                                 [pos[2] + params['sample_retract_dist']]),
                                params['lift_speed'])
                    # Calculate result
                    epos = probe.calc_probe_z_average(positions,
                                                      params['samples_result'])
                # self.results.append(epos)
                return epos
            else:
                return self._probe(speed, allow_faulty)
        finally:
            self._stop_streaming()

    def _move_to_probing_height(self, speed):
        target = self.trigger_distance
        top = target + self.backlash_comp
        cur_z = self.toolhead.get_position()[2]
        if cur_z < top:
            self.toolhead.manual_move([None, None, top], speed)
        self.toolhead.manual_move([None, None, target], speed)
        self.toolhead.wait_moves()

    def _probing_move_to_probing_height(self, speed):
        curtime = self.reactor.monotonic()
        status = self.toolhead.get_kinematics().get_status(curtime)
        pos = self.toolhead.get_position()
        pos[2] = status["axis_minimum"][2]
        try:
            self.phoming.probing_move(self.mcu_probe, pos, speed)
            self._sample_printtime_sync(self.z_settling_time)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during probing move" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)

    # TODO: 探针测试
    def _probe_2(self, speed, pos, count=0):
        # toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in self.toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        try:
            epos = self.mcu_probe.probing_move(pos, speed)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # Allow axis_twist_compensation to update results
        self.printer.send_event("probe:update_results", epos)
        # Report results
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                           % (epos[0], epos[1], epos[2]))
        return epos[:3]

    def _probe(self, speed, num_samples=10, allow_faulty=False):
        target = self.trigger_distance
        tdt = self.trigger_dive_threshold
        (dist, samples) = self._sample(5, num_samples)

        x, y = samples[0]["pos"][0:2]
        if self._is_faulty_coordinate(x, y, True):
            msg = "Probing within a faulty area"
            if not allow_faulty:
                raise self.printer.command_error(msg)
            else:
                self.gcode.respond_raw("!! " + msg + "\n")

        if dist > target + tdt:
            # If we are above the dive threshold right now, we'll need to
            # do probing move and then re-measure
            self._probing_move_to_probing_height(speed)
            (dist, samples) = self._sample(self.z_settling_time, num_samples)
        elif math.isinf(dist) and dist < 0:
            # We were below the valid range of the model
            msg = """Attempted to probe with
                MD_DIST below calibrated model range"""
            raise self.printer.command_error(msg)
        elif self.toolhead.get_position()[2] < target - tdt:
            # We are below the probing target height, we'll move to the
            # correct height and take a new sample.
            self._move_to_probing_height(speed)
            (dist, samples) = self._sample(self.z_settling_time, num_samples)

        pos = samples[0]["pos"]

        self.gcode.respond_info(
            "probe at %.3f,%.3f,%.3f is z=%.6f"
            % (pos[0], pos[1], pos[2], dist)
        )

        return [pos[0], pos[1], pos[2] + target - dist]

    # Calibration routines
    def _start_calibration(self, gcmd):
        if self.z_virtual_endstop:
            allow_faulty = gcmd.get_int("ALLOW_FAULTY_COORDINATE", 0) != 0
            if gcmd.get("SKIP_MANUAL_PROBE", None) is not None:
                kin = self.toolhead.get_kinematics()
                kin_spos = {
                    s.get_name(): s.get_commanded_position() \
                        for s in kin.get_steppers()
                }
                kin_pos = kin.calc_position(kin_spos)
                if self._is_faulty_coordinate(kin_pos[0], kin_pos[1]):
                    msg = "Calibrating within a faulty area"
                    if not allow_faulty:
                        raise gcmd.error(msg)
                    else:
                        gcmd.respond_raw("!! " + msg + "\n")
                self._calibrate(gcmd, kin_pos, False)
            else:
                curtime = self.printer.get_reactor().monotonic()
                kin_status = self.toolhead.get_status(curtime)
                if "xy" not in kin_status["homed_axes"]:
                    raise self.printer.command_error(
                        "Must home X and Y before calibration"
                    )

                kin_pos = self.toolhead.get_position()
                if self._is_faulty_coordinate(kin_pos[0], kin_pos[1]):
                    msg = "Calibrating within a faulty area"
                    if not allow_faulty:
                        raise gcmd.error(msg)
                    else:
                        gcmd.respond_raw("!! " + msg + "\n")

                forced_z = False
                if 'z' not in kin_status["homed_axes"]:
                    self.toolhead.get_last_move_time()
                    pos = self.toolhead.get_position()
                    pos[2] = kin_status["axis_maximum"][2] - 1.0
                    self.toolhead.set_position(pos, homing_axes=[2])
                    forced_z = True

                cb = lambda kin_pos: self._calibrate(gcmd, kin_pos, forced_z)
                manual_probe.ManualProbeHelper(self.printer, gcmd, cb)
        else:
            curtime = self.printer.get_reactor().monotonic()
            kin_status = self.toolhead.get_status(curtime)
            if "xyz" not in kin_status["homed_axes"]:
                raise self.printer.command_error(
                    "Must home before calibration"
                )
            kin_pos = self.toolhead.get_position()
            if self._is_faulty_coordinate(kin_pos[0], kin_pos[1]):
                msg = "Calibrating within a faulty area"
                if not allow_faulty:
                    raise gcmd.error(msg)
                else:
                    gcmd.respond_raw("!! " + msg + "\n")
            forced_z = False
            # kin_pos[2] = self.cal_nozzle_z
            self._calibrate(gcmd, kin_pos, forced_z)

    def _calibrate(self, gcmd, kin_pos, forced_z):
        if kin_pos is None:
            if forced_z:
                kin = self.toolhead.get_kinematics()
                if hasattr(kin, "note_z_not_homed"):
                    kin.note_z_not_homed()
            return

        gcmd.respond_info("MD_DIST calibration starting")
        cal_nozzle_z = gcmd.get_float("NOZZLE_Z", self.cal_nozzle_z)
        cal_floor = gcmd.get_float("FLOOR", self.cal_floor)
        cal_ceil = gcmd.get_float("CEIL", self.cal_ceil)
        cal_min_z = kin_pos[2] - cal_nozzle_z + cal_floor
        cal_max_z = kin_pos[2] - cal_nozzle_z + cal_ceil
        cal_speed = gcmd.get_float("SPEED", self.cal_speed)
        move_speed = gcmd.get_float("MOVE_SPEED", self.cal_move_speed)

        toolhead = self.toolhead
        curtime = self.reactor.monotonic()
        toolhead.wait_moves()
        pos = toolhead.get_position()

        # Move over to probe coordinate and pull out backlash
        curpos = self.toolhead.get_position()

        curpos[2] = cal_max_z + self.backlash_comp
        toolhead.manual_move(curpos, move_speed) # Up
        curpos[0] -= self.x_offset
        curpos[1] -= self.y_offset
        toolhead.manual_move(curpos, move_speed) # Over
        curpos[2] = cal_max_z
        toolhead.manual_move(curpos, move_speed) # Down
        toolhead.wait_moves()

        samples = []
        def cb(sample):
            samples.append(sample)

        try:
            # TODO: 定位,获取流
            self._start_streaming()
            self._sample_printtime_sync(50)
            with self.streaming_session(cb) as ss:
                self._sample_printtime_sync(50)
                toolhead.dwell(0.250)
                curpos[2] = cal_min_z
                toolhead.manual_move(curpos, cal_speed)
                toolhead.dwell(0.250)
                self._sample_printtime_sync(50)
        finally:
            self._stop_streaming()

        # Fit the sampled data
        z_offset = [s["pos"][2]-cal_min_z+cal_floor for s in samples]
        freq = [s["freq"] for s in samples]
        temp = [s["temp"] for s in samples]
        inv_freq = [1/f for f in freq]
        poly = Polynomial.fit(inv_freq, z_offset, 9)
        temp_median = median(temp)
        self.model = MD_Dist_Model(
            "default", self, poly, temp_median, min(z_offset), max(z_offset)
        )
        self.models[self.model.name] = self.model
        self.model.save(self)
        self._apply_threshold()

        self.toolhead.get_last_move_time()
        pos = self.toolhead.get_position()
        pos[2] = cal_floor
        self.toolhead.set_position(pos)

        # Dump calibration curve
        fn = "/tmp/md_dist_calibrate-"+time.strftime("%Y%m%d_%H%M%S")+".csv"
        f = open(fn, "w")
        f.write("freq,z,temp\n")
        for i in range(len(freq)):
            f.write("%.5f,%.5f,%.3f\n" % (freq[i], z_offset[i], temp[i]))
        f.close()

        gcmd.respond_info(
            "MD_DIST calibrated at %.3f,%.3f from "
            "%.3f to %.3f, speed %.2f mm/s, temp %.2fC"
            % (pos[0], pos[1], cal_min_z, cal_max_z, cal_speed, temp_median)
        )

    # Internal
    def _update_thresholds(self, moving_up=False, freq=None):
        if self.version_int == 300:
            if freq is None:
                return
            self.trigger_freq = freq
        else:
            self.trigger_freq = self.dist_to_freq(
                self.trigger_distance,
                self.last_temp
            )
        self.untrigger_freq = self.trigger_freq * (1-self.trigger_hysteresis)
        logging.info(
            "_update_thresholds trigger_freq: %.3f, untrigger_freq: %.3f"
            % (self.trigger_freq, self.untrigger_freq)
        )

    def _apply_threshold(self, moving_up=False, freq=None):
        self._update_thresholds(moving_up, freq)
        trigger_c = int(self.freq_to_count(self.trigger_freq))
        untrigger_c = int(self.freq_to_count(self.untrigger_freq))
        self.md_dist_set_threshold_cmd.send([trigger_c, untrigger_c])

    def _register_model(self, name, model):
        if name in self.models:
            raise self.printer.config_error(
                "Multiple md_dist models with same name '%s'" % (name,)
            )
        self.models[name] = model

    def _is_faulty_coordinate(self, x, y, add_offsets=False):
        if not self.mesh_helper:
            return False
        return self.mesh_helper._is_faulty_coordinate(x, y, add_offsets)

    # Streaming mode
    def _check_hardware(self, sample):
        if not self.hardware_failure:
            msg = None
            if sample["data"] == 0xFFFFFFF:
                msg = "coil is shorted or not connected"
            elif self.fmin is not None and sample["freq"] > 1.35*self.fmin:
                msg = "coil expected max frequency exceeded"
            if msg:
                msg = "MD_DIST hardware issue: " + msg
                self.hardware_failure = msg
                logging.error(msg)
                if self._stream_en:
                    self.printer.invoke_shutdown(msg)
                else:
                    self.gcode.respond_raw("!! " + msg + "\n")
        elif self._stream_en:
            self.printer.invoke_shutdown(self.hardware_failure)

    def _enrich_sample_time(self, sample):
        clock = sample["clock"] = self._mcu.clock32_to_clock64(
            sample["clock"]
        )
        sample["time"] = self._mcu.clock_to_print_time(clock)

    def _enrich_sample_temp(self, sample):
        if self.use_known_temp_sensor:
            current_temp, target_temp = self.sensor_obj.get_temp(sample["time"])
            sample["temp"] = current_temp
        else:
            temp_adc = (sample["temp"] / self.temp_smooth_count *
                        self.inv_adc_max)
            sample["temp"] = self.thermistor.calc_temp(temp_adc)

    def _enrich_sample_freq(self, sample):
        sample["data_smooth"] = self._data_filter.value()
        sample["freq"] = self.count_to_freq(sample["data_smooth"])
        self._check_hardware(sample)

    def _enrich_sample(self, sample):
        sample["dist"] = self.freq_to_dist(sample["freq"], sample["temp"])
        pos, vel = self._get_trapq_position(sample["time"])

        if pos is None:
            return
        sample["pos"] = pos
        sample["vel"] = vel

    def _start_streaming(self):
        if self._stream_en == 0:
            self.md_dist_stream_cmd.send([1])
            curtime = self.reactor.monotonic()
            self.reactor.update_timer(
                self._stream_timeout_timer,
                curtime + STREAM_TIMEOUT
            )
            logging.info("MD_DIST send stream 1")
        self._stream_en += 1
        self._data_filter.reset()
        self._stream_flush()
    def _stop_streaming(self):
        self._stream_en -= 1
        if self._stream_en == 0:
            self.reactor.update_timer(
                self._stream_timeout_timer,
                self.reactor.NEVER
            )
            self.md_dist_stream_cmd.send([0])
            logging.info("MD_DIST send stream 0")
        self._stream_flush()

    def _stream_timeout(self, eventtime):
        if not self._stream_en:
            return self.reactor.NEVER
        msg = "MD_DIST sensor not receiving data"
        logging.error(msg)
        self.printer.invoke_shutdown(msg)
        return self.reactor.NEVER

    def request_stream_latency(self, latency):
        next_key = 0
        if self._stream_latency_requests:
            next_key = max(self._stream_latency_requests.keys()) + 1
        new_limit = STREAM_BUFFER_LIMIT_DEFAULT
        self._stream_latency_requests[next_key] = latency
        min_requested = min(self._stream_latency_requests.values())
        if min_requested < new_limit:
            new_limit = min_requested
        if new_limit < 1:
            new_limit = 1
        self._stream_buffer_limit_new = new_limit
        return next_key

    def drop_stream_latency_request(self, key):
        self._stream_latency_requests.pop(key, None)
        new_limit = STREAM_BUFFER_LIMIT_DEFAULT
        if self._stream_latency_requests:
            min_requested = min(self._stream_latency_requests.values())
            if min_requested < new_limit:
                new_limit = min_requested
        if new_limit < 1:
            new_limit = 1
        self._stream_buffer_limit_new = new_limit

    def streaming_session(self, callback,
                          completion_callback=None,
                          latency=None):
        return Streaming_Helper(self, callback, completion_callback, latency)

    def _stream_flush(self):
        self._stream_flush_event.clear()
        while True:
            try:
                samples = self._stream_samples_queue.get_nowait()
                updated_timer = False
                for sample in samples:
                    if not updated_timer:
                        curtime = self.reactor.monotonic()
                        self.reactor.update_timer(
                            self._stream_timeout_timer,
                            curtime + STREAM_TIMEOUT
                        )
                        updated_timer = True

                    self._enrich_sample_time(sample)
                    self._data_filter.update(sample["time"], sample["data"])

                    if self.enable_temp_compensate:
                        self._enrich_sample_temp(sample)
                        temp = sample["temp"]
                        if (self.model_temp is not None and
                            not (-40 < temp < 180)):
                            msg = (
                                "MD_DIST temperature sensor faulty"
                                "(read %.2f C),"
                                " disabling temperaure compensation" %
                                (temp,)
                            )
                            logging.error(msg)
                            self.gcode.respond_raw("!! " + msg + "\n")
                            self.model_temp = None

                        self.last_temp = temp
                        if temp:
                            self.measured_min = min(self.measured_min, temp)
                            self.measured_max = max(self.measured_max, temp)

                    self._enrich_sample_freq(sample)

                    if len(self._stream_callbacks) > 0:
                        self._enrich_sample(sample)
                        for cb in list(self._stream_callbacks.values()):
                            cb(sample)
            except queue.Empty:
                return

    def _stream_flush_schedule(self):
        # When streaming is disabled, let all through
        force = self._stream_en == 0
        if self._stream_buffer_limit_new != self._stream_buffer_limit:
            force = True
            self._stream_buffer_limit = self._stream_buffer_limit_new
        if not force and len(self._stream_buffer) < self._stream_buffer_limit:
            return
        self._stream_samples_queue.put_nowait(self._stream_buffer)
        self._stream_buffer = []
        if self._stream_flush_event.is_set():
            return
        self._stream_flush_event.set()
        self.reactor.register_async_callback(lambda e: self._stream_flush())

    def _handle_md_dist_data(self, params):
        if self.trapq is None:
            logging.info("md_dist: no_trapq")
            return

        # logging.info("md_dist: data=%s temp=%s" %
        # self.gcode.respond_info("md_dist: data=%s temp=%s" %
            #    (params["data"], params["temp"],))

        self._stream_buffer.append(params.copy())
        self._stream_flush_schedule()

    def _get_trapq_position(self, print_time):
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new("struct pull_move[1]")
        count = ffi_lib.trapq_extract_old(
            self.trapq, data, 1, 0.0, print_time
        )
        if not count:
            return None, None
        move = data[0]
        move_time = max(0.0, min(move.move_t, print_time - move.print_time))
        dist = (move.start_v + 0.5 * move.accel * move_time) * move_time
        pos = (
            move.start_x + move.x_r * dist,
            move.start_y + move.y_r * dist,
            move.start_z + move.z_r * dist
        )
        velocity = move.start_v + move.accel * move_time
        return pos, velocity

    def _sample_printtime_sync(self, skip=0, count=1):
        toolhead = self.printer.lookup_object("toolhead")
        move_time = toolhead.get_last_move_time()
        settle_clock = self._mcu.print_time_to_clock(move_time)
        samples = []
        total = skip + count

        def cb(sample):
            if sample["clock"] >= settle_clock:
                samples.append(sample)
                if len(samples) >= total:
                    raise Stop_Streaming

        with self.streaming_session(cb, latency=skip+count) as ss:
            ss.wait()

        samples = samples[skip:]

        if count == 1:
            return samples[0]
        else:
            return samples

    def _sample(self, skip, count):
        samples = self._sample_printtime_sync(skip, count)
        return (median([s["dist"] for s in samples]), samples)

    def _sample_async(self, count=1):
        samples = []
        def cb(sample):
            samples.append(sample)
            if len(samples) >= count:
                raise Stop_Streaming

        with self.streaming_session(cb, latency=count) as ss:
            ss.wait()

        if count == 1:
            return samples[0]
        else:
            return samples

    def count_to_freq(self, count):
        return count*self.sensor_freq/(2**28)

    def freq_to_count(self, freq):
        return freq*(2**28)/self.sensor_freq

    def dist_to_freq(self, dist, temp):
        if self.model is None:
            return None
        return self.model.dist_to_freq(dist, temp)

    def freq_to_dist(self, freq, temp):
        if self.model is None:
            return None
        return self.model.freq_to_dist(freq, temp)

    def get_status(self, eventtime):
        model = None
        if self.model is not None:
            model = self.model.name
        x_offset, y_offset, z_offset = self.get_offsets()
        return {
            'name': self.name,
            'last_sample': self.last_sample,
            'model': model,
            'offsets': {'x': x_offset, 'y': y_offset, 'z': z_offset}
        }

    # Webhook handlers

    def _handle_req_status(self, web_request):
        temp = None
        sample = self._sample_async()
        out = {
            "freq": sample["freq"],
            "dist": sample["dist"],
        }
        temp = sample["temp"]
        if temp is not None:
            out["temp"] = temp
        web_request.send(out)

    def _handle_req_dump(self, web_request):
        self._api_dump_helper.add_client(web_request)

    # GCode command handlers

    cmd_PROBE_help = "Probe Z-height at current XY position"
    def cmd_PROBE(self, gcmd):
        pos = self.run_probe(gcmd)
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))

    cmd_MD_DIST_CALIBRATE_help = "Calibrate MD_DIST response curve"
    def cmd_MD_DIST_CALIBRATE(self,gcmd):
        self._start_calibration(gcmd)

    cmd_MD_DIST_ESTIMATE_BACKLASH_help = "Estimate Z axis backlash"
    def cmd_MD_DIST_ESTIMATE_BACKLASH(self, gcmd):
        # Get to correct Z height
        overrun = gcmd.get_float("OVERRUN", 1.0)
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        cur_z = self.toolhead.get_position()[2]
        self.toolhead.manual_move([None, None, cur_z+overrun], speed)
        self.run_probe(gcmd)

        lift_speed = self.get_lift_speed(gcmd)
        target = gcmd.get_float('Z', self.trigger_distance)

        num_samples = gcmd.get_int("SAMPLES", 20)
        wait = self.z_settling_time

        samples_up = []
        samples_down = []

        next_dir = -1

        try:
            self._start_streaming()

            (cur_dist, _samples) = self._sample(wait, 10)
            pos = self.toolhead.get_position()
            missing = target - cur_dist
            target = pos[2] + missing
            gcmd.respond_info("Target kinematic Z is %.3f" % (target,))

            if target - overrun < 0:
                raise gcmd.error("Target minus overrun must exceed 0mm")

            while len(samples_up) + len(samples_down) < num_samples:
                liftpos = [None, None, target + overrun * next_dir]
                self.toolhead.manual_move(liftpos, lift_speed)
                liftpos = [None, None, target]
                self.toolhead.manual_move(liftpos, lift_speed)
                self.toolhead.wait_moves()
                (dist, _samples) = self._sample(wait, 10)
                {-1: samples_up, 1: samples_down}[next_dir].append(dist)
                next_dir = next_dir * -1

        finally:
            self._stop_streaming()

        res_up = median(samples_up)
        res_down = median(samples_down)

        gcmd.respond_info(
            "Median distance moving up %.5f, down %.5f, "
            "delta %.5f over %d samples"
            % (res_up, res_down, res_down - res_up, num_samples)
        )

    cmd_MD_DIST_QUERY_help = "Take a sample from the sensor"
    def cmd_MD_DIST_QUERY(self, gcmd):
        sample = self._sample_async()
        last_value = sample["freq"]
        dist = sample["dist"]
        temp = sample["temp"]
        self.last_sample = {
            "time": sample["time"],
            "value": last_value,
            "temp": temp,
            "dist": dist,
        }
        if dist is None:
            gcmd.respond_info(
                "Last reading: %.2fHz, %.2fC, no model"
                % (last_value, temp,)
            )
        else:
            gcmd.respond_info(
                "Last reading: %.2fHz, %.2fC, %.5fmm"
                % (last_value, temp, dist)
            )

    cmd_MD_DIST_STREAM_help = "Enable MD_DIST Streaming"
    def cmd_MD_DIST_STREAM(self, gcmd):
        if self._log_stream is not None:
            self._log_stream.stop()
            self._log_stream = None
            gcmd.respond_info("MD_DIST Streaming disabled")
        else:
            f = None
            completion_cb = None
            fn = gcmd.get("FILENAME")
            f = open(fn, "w")
            def close_file():
                f.close()
            completion_cb = close_file
            f.write("time,data,data_smooth,freq,dist,\
                     temp,pos_x,pos_y,pos_z,vel\n")

            def cb(sample):
                pos = sample.get("pos", None)
                obj = "%.4f,%d,%.2f,%.5f,%.5f,%.2f,%s,%s,%s,%s\n" % (
                    sample["time"],
                    sample["data"],
                    sample["data_smooth"],
                    sample["freq"],
                    sample["dist"],
                    sample["temp"],
                    "%.3f" % (pos[0],) if pos is not None else "",
                    "%.3f" % (pos[1],) if pos is not None else "",
                    "%.3f" % (pos[2],) if pos is not None else "",
                    "%.3f" % (sample["vel"],) if "vel" in sample else ""
                )
                f.write(obj)

            self._log_stream = self.streaming_session(cb, completion_cb)
            gcmd.respond_info("MD_DIST Streaming enabled")

    cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"
    def cmd_PROBE_ACCURACY(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", 0)
        allow_faulty = gcmd.get_int("ALLOW_FAULTY_COORDINATE", 0) != 0
        pos = self.toolhead.get_position()
        gcmd.respond_info(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
            " (samples=%d retract=%.3f"
            " speed=%.1f lift_speed=%.1f)\n"
            % (pos[0], pos[1], pos[2],
                sample_count, sample_retract_dist,
                speed, lift_speed)
        )

        start_height = self.trigger_distance + sample_retract_dist
        liftpos = [None, None, start_height]
        self.toolhead.manual_move(liftpos, lift_speed)

        self.multi_probe_begin()
        positions = []
        while len(positions) < sample_count:
            pos = self._probe(speed, allow_faulty=allow_faulty)
            positions.append(pos)
            self.toolhead.manual_move(liftpos, lift_speed)
        self.multi_probe_end()

        zs = [p[2] for p in positions]
        max_value = max(zs)
        min_value = min(zs)
        range_value = max_value - min_value
        avg_value = sum(zs) / len(positions)
        median_ = median(zs)

        deviation_sum = 0
        for i in range(len(zs)):
            deviation_sum += pow(zs[2] - avg_value, 2.0)
        sigma = (deviation_sum / len(zs)) ** 0.5

        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f"
            % (max_value, min_value, range_value, avg_value, median_, sigma))

    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"
    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        gcode_move = self.printer.lookup_object("gcode_move")
        offset = gcode_move.get_status()["homing_origin"].z

        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
            return

        if not self.model:
            raise self.gcode.error("You must calibrate your model first, "
                                   "use MD_DIST_CALIBRATE.")

        # We use the model code to save the new offset, but we can't actually
        # apply that offset yet because the gcode_offset is still in effect.
        # If the user continues to do stuff after this, the newly set model
        # offset would compound with the gcode offset. To ensure this doesn't
        # happen, we revert to the old model offset afterwards.
        # Really, the user should just be calling `SAVE_CONFIG` now.
        old_offset = self.model.offset
        self.model.offset += offset
        self.model.save(self, False)
        gcmd.respond_info(
            "md_dist model offset has been updated\n"
            "You must run the SAVE_CONFIG command now to update the\n"
            "printer config file and restart the printer."
        )
        self.model.offset = old_offset

class MD_Dist_Model:
    @classmethod
    def load(cls, name, config, md_dist):
        coef = config.getfloatlist("model_coef")
        temp = config.getfloat("model_temp")
        domain = config.getfloatlist("model_domain", count=2)
        [min_z, max_z] = config.getfloatlist("model_range", count=2)
        offset = config.getfloat("model_offset", 0.0)
        poly = Polynomial(coef, domain)
        return MD_Dist_Model(name, md_dist, poly, temp, min_z, max_z, offset)

    def __init__(self, name, md_dist, poly, temp, min_z, max_z, offset=0):
        self.name = name
        self.md_dist = md_dist
        self.poly = poly
        self.min_z = min_z
        self.max_z = max_z
        self.temp = temp
        self.offset = offset
        self.enable_temp_compensate = md_dist.enable_temp_compensate

    def save(self, md_dist, show_message=True):
        configfile = md_dist.printer.lookup_object("configfile")
        section = "md_dist model " + self.name
        configfile.set(section, "model_coef",
                       ",\n  ".join(map(str, self.poly.coef)))
        configfile.set(section, "model_domain",
                       ",".join(map(str, self.poly.domain)))
        configfile.set(section, "model_range",
                       "%f,%f" % (self.min_z, self.max_z))
        configfile.set(section, "model_temp",
                       "%f" % (self.temp))
        configfile.set(section, "model_offset", "%.5f" % (self.offset,))
        if show_message:
            md_dist.gcode.respond_info(
                "MD_DIST calibration for model '%s' has "
                "been updated\nfor the current session. The SAVE_CONFIG "
                "command will\nupdate the printer config file and restart "
                "the printer." % (self.name,)
            )

    def freq_to_dist_raw(self, freq):
        [begin, end] = self.poly.domain
        invfreq = 1/freq
        if invfreq > end:
            return float("inf")
        elif invfreq < begin:
            return float("-inf")
        else:
            return float(self.poly(invfreq) - self.offset)

    def freq_to_dist(self, freq, temp):
        if (self.temp is not None
            and self.md_dist.model_temp is not None
            and self.enable_temp_compensate):
            freq = self.md_dist.model_temp.compensate(freq, temp, self.temp)
        return self.freq_to_dist_raw(freq)

    def dist_to_freq_raw(self, dist, max_e=0.00000001):
        if dist < self.min_z or dist > self.max_z:
            msg = ("Attempted to map out-of-range distance %f, valid range "
                   "[%.3f, %.3f]" % (dist, self.min_z, self.max_z))
            raise self.md_dist.printer.command_error(msg)
        dist += self.offset
        [begin, end] = self.poly.domain
        for _ in range(0, 50):
            f = (end + begin) / 2
            v = self.poly(f)
            if abs(v-dist) < max_e:
                return float(1.0/f)
            elif v < dist:
                begin = f
            else:
                end = f
        raise self.md_dist.printer.command_error(
                "md_dist model convergence error")

    def dist_to_freq(self, dist, temp, max_e=0.00000001):
        freq = self.dist_to_freq_raw(dist, max_e)
        if (self.temp is not None
            and self.md_dist.model_temp is not None
            and self.enable_temp_compensate):
            freq = self.md_dist.model_temp.compensate(freq, self.temp, temp)
        return freq

class MD_Dist_Temp_Model_Builder:
    _DEFAULTS = {
        "amfg": 1.0,
        "tcc": -1.56165495e-05,
        "tcfl": -1.11115902e-12,
        "tctl": 3.6738370e-16,
        "fmin" : None,
        "fmin_temp" : None
    }

    @classmethod
    def load(cls, config):
        return MD_Dist_Temp_Model_Builder(config)

    def __init__(self, config):
        self.parameters = MD_Dist_Temp_Model_Builder._DEFAULTS.copy()
        for key in self.parameters.keys():
            param = config.getfloat("tc_" + key, None)
            if param is not None:
                self.parameters[key] = param

    def build(self):
        if self.parameters["fmin"] is None \
           or self.parameters["fmin_temp"] is None:
            return None
        logging.info("MD_DIST: built tempco model %s", self.parameters)
        return MD_Dist_Temp_Model(**self.parameters)

    def build_with_base(self, md_dist):
        base_data = md_dist.md_dist_base_read_cmd.send([6, 0])
        (f_count, adc_count) = struct.unpack("<IH", base_data["bytes"])
        if f_count < 0xFFFFFFFF and adc_count < 0xFFFF:
            if self.parameters["fmin"] is None:
                self.parameters["fmin"] = md_dist.count_to_freq(f_count)
                logging.info(
                    "MD_DIST: loaded fmin=%.2f from base",
                    self.parameters["fmin"]
                )
            if self.parameters["fmin_temp"] is None:
                temp_adc = float(adc_count) / md_dist.temp_smooth_count * \
                           md_dist.inv_adc_max
                self.parameters["fmin_temp"] = md_dist.thermistor.calc_temp(
                    temp_adc
                )
                logging.info(
                    "MD_DIST: loaded fmin_temp=%.2f from base",
                    self.parameters["fmin_temp"]
                )
        else:
            logging.info("MD_DIST: fmin parameters not found in base")
        return self.build()

class MD_Dist_Temp_Model:
    def __init__(self, amfg, tcc, tcfl, tctl, fmin, fmin_temp):
        self.amfg = amfg
        self.tcc = tcc
        self.tcfl = tcfl
        self.tctl = tctl
        self.fmin = fmin
        self.fmin_temp = fmin_temp

    def _tcf(self, f, df, dt, tctl):
        tctl = self.tctl if tctl is None else tctl
        tc = self.tcc + self.tcfl * df + tctl * df * df
        return f + self.amfg * tc * dt * f

    def compensate(self, freq, temp_source, temp_target, tctl=None):
        dt = temp_target - temp_source
        dfmin = self.fmin * self.amfg * self.tcc * \
                (temp_source - self.fmin_temp)
        df = freq - (self.fmin + dfmin)
        if dt < 0.:
            f2 = self._tcf(freq, df, dt, tctl)
            dfmin2 = self.fmin * self.amfg * self.tcc * \
                    (temp_target - self.fmin_temp)
            df2 = f2 - (self.fmin + dfmin2)
            f3 = self._tcf(f2, df2, -dt, tctl)
            ferror = freq - f3
            freq = freq + ferror
            df = freq - (self.fmin + dfmin)
        return self._tcf(freq, df, dt, tctl)

class Model_Manager:
    def __init__(self, md_dist):
        self.md_dist = md_dist
        self.gcode = md_dist.printer.lookup_object("gcode")
        self.gcode.register_command(
            "MD_DIST_MODEL_SELECT",
            self.cmd_MD_DIST_MODEL_SELECT,
            desc=self.cmd_MD_DIST_MODEL_SELECT_help
        )
        self.gcode.register_command(
            "MD_DIST_MODEL_SAVE",
            self.cmd_MD_DIST_MODEL_SAVE,
            desc=self.cmd_MD_DIST_MODEL_SAVE_help
        )
        self.gcode.register_command(
            "MD_DIST_MODEL_REMOVE",
            self.cmd_MD_DIST_MODEL_REMOVE,
            desc=self.cmd_MD_DIST_MODEL_REMOVE_help
        )
        self.gcode.register_command(
            "MD_DIST_MODEL_LIST",
            self.cmd_MD_DIST_MODEL_LIST,
            desc=self.cmd_MD_DIST_MODEL_LIST_help
        )

    cmd_MD_DIST_MODEL_SELECT_help = "Load named md_dist model"
    def cmd_MD_DIST_MODEL_SELECT(self, gcmd):
        name = gcmd.get("NAME")
        model = self.md_dist.models.get(name, None)
        if model is None:
            raise gcmd.error("Unknown model '%s'" % (name,))
        self.md_dist.model = model
        gcmd.respond_info("Selected md_dist model '%s'" % (name,))

    cmd_MD_DIST_MODEL_SAVE_help = "Save current md_dist model"
    def cmd_MD_DIST_MODEL_SAVE(self, gcmd):
        model = self.md_dist.model
        if model is None:
            raise gcmd.error("No model currently selected")
        oldname = model.name
        name = gcmd.get("NAME", oldname)
        if name != oldname:
            model = copy.copy(model)
        model.name = name
        model.save(self.md_dist)
        if name != oldname:
            self.md_dist.models[name] = model

    cmd_MD_DIST_MODEL_REMOVE_help = "Remove saved md_dist model"
    def cmd_MD_DIST_MODEL_REMOVE(self, gcmd):
        name = gcmd.get("NAME")
        model = self.md_dist.models.get(name, None)
        if model is None:
            raise gcmd.error("Unknown model '%s'" % (name,))
        configfile = self.md_dist.printer.lookup_object("configfile")
        section = "md_dist model " + model.name
        configfile.remove_section(section)
        self.md_dist.models.pop(name)
        gcmd.respond_info(
            "Model '%s' was removed for the current session.\n"
            "Run SAVE_CONFIG to update the printer configuration"
            "and restart Klipper." % (name,)
        )
        if self.md_dist.model == model:
            self.md_dist.model = None

    cmd_MD_DIST_MODEL_LIST_help = "Remove saved md_dist model"
    def cmd_MD_DIST_MODEL_LIST(self, gcmd):
        if not self.md_dist.models:
            gcmd.respond_info("No md_dist models loaded")
            return
        gcmd.respond_info("List of loaded md_dist models:")
        current_model = self.md_dist.model
        for _name, model in sorted(self.md_dist.models.items()):
            if model == current_model:
                gcmd.respond_info("- %s [active]" % (model.name,))
            else:
                gcmd.respond_info("- %s" % (model.name,))


class Alpha_Beta_Filter:
    def __init__(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta
        self.reset()

    def reset(self):
        self.xl = None
        self.vl = 0
        self.tl = None

    def update(self, time, measurement):
        if self.xl == None:
            self.xl = measurement
        if self.tl is not None:
            dt = time - self.tl
        else:
            dt = 0
        self.tl = time
        xk = self.xl + self.vl * dt
        vk = self.vl
        rk = measurement - xk
        xk = xk + self.alpha * rk
        if dt > 0:
            vk = vk + self.beta / dt * rk
        self.xl = xk
        self.vl = vk
        return xk

    def value(self):
        return self.xl

class Streaming_Helper:
    def __init__(self, md_dist, callback, completion_callback, latency):
        self.md_dist = md_dist
        self.cb = callback
        self.completion_cb = completion_callback
        self.completion = self.md_dist.reactor.completion()

        self.latency_key = None
        if latency is not None:
            self.latency_key = self.md_dist.request_stream_latency(latency)

        self.md_dist._stream_callbacks[self] = self._handle
        self.md_dist._start_streaming()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def _handle(self, sample):
        try:
            self.cb(sample)
        except Stop_Streaming:
            self.completion.complete(())

    def stop(self):
        if not self in self.md_dist._stream_callbacks:
            return
        del self.md_dist._stream_callbacks[self]
        self.md_dist._stop_streaming()
        if self.latency_key is not None:
            self.md_dist.drop_stream_latency_request(self.latency_key)
        if self.completion_cb is not None:
            self.completion_cb()

    def wait(self):
        self.completion.wait()
        self.stop()

class Stop_Streaming(Exception):
    pass


class API_Dump_Helper:
    def __init__(self, md_dist):
        self.md_dist = md_dist
        self.clients = {}
        self.stream = None
        self.buffer = []
        self.fields = ["dist", "temp", "pos", "freq", "vel", "time"]

    def _start_stop(self):
        if not self.stream and self.clients:
            self.stream = self.md_dist.streaming_session(self._cb)
        elif self.stream is not None and not self.clients:
            self.stream.stop()
            self.stream = None

    def _cb(self, sample):
        tmp = [sample.get(key, None) for key in self.fields]
        self.buffer.append(tmp)
        if len(self.buffer) > 50:
            self._update_clients()

    def _update_clients(self):
        for cconn, template in list(self.clients.items()):
            if cconn.is_closed():
                del self.clients[cconn]
                self._start_stop()
                continue
            tmp = dict(template)
            tmp["params"] = self.buffer
            cconn.send(tmp)
        self.buffer = []

    def add_client(self, web_request):
        cconn = web_request.get_client_connection()
        template = web_request.get_dict("response_template", {})
        self.clients[cconn] = template
        self._start_stop()
        web_request.send({"header": self.fields})

class MD_Dist_Probe_Wrapper:
    def __init__(self, md_dist):
        self.md_dist = md_dist

    def multi_probe_begin(self):
        logging.info("M_D: multi_probe_begin1")
        return self.md_dist.multi_probe_begin()
    def multi_probe_end(self):
        logging.info("M_D: multi_probe_end1")
        return self.md_dist.multi_probe_end()
    def get_offsets(self):
        return self.md_dist.get_offsets()
    def get_lift_speed(self, gcmd=None):
        return self.md_dist.get_lift_speed(gcmd)
    def run_probe(self, gcmd):
        return self.md_dist.run_probe(gcmd)
    def get_status(self, eventtime):
        return self.md_dist.get_status(eventtime)

class MD_Dist_Temp_Wrapper:
    def __init__(self, md_dist):
        self.md_dist = md_dist

    def get_temp(self, eventtime):
        return self.md_dist.last_temp, 0

    def get_status(self, eventtime):
        return {
            "temperature": round(self.md_dist.last_temp, 2),
            "measured_min_temp": round(self.md_dist.measured_min, 2),
            "measured_max_temp": round(self.md_dist.measured_max, 2)
        }

TRSYNC_TIMEOUT = 0.025
TRSYNC_SINGLE_MCU_TIMEOUT = 0.250

class MD_Dist_Endstop_Wrapper:
    def __init__(self, md_dist):
        self.md_dist = md_dist
        self._mcu = md_dist._mcu
        self.version_int = self.md_dist.version_int

        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(
            ffi_lib.trdispatch_alloc(),
            ffi_lib.free
        )
        self._trsyncs = [MCU_trsync(self.md_dist._mcu, self._trdispatch)]

        printer = self.md_dist.printer
        printer.register_event_handler(
            "klippy:mcu_identify",
            self._handle_mcu_identify
        )
        printer.register_event_handler(
            "homing:home_rails_begin",
            self._handle_home_rails_begin
        )
        printer.register_event_handler(
            "homing:home_rails_end",
            self._handle_home_rails_end
        )

        self.z_homed = False
        self.is_homing = False

    def _handle_mcu_identify(self):
        self.toolhead = self.md_dist.printer.lookup_object("toolhead")
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def _handle_home_rails_begin(self, homing_state, rails):
        self.is_homing = False

    def _handle_home_rails_end(self, homing_state, rails):
        if (self.md_dist.model is None) and (self.version_int != 300):
            return

        if not self.is_homing:
            return

        if 2 not in homing_state.get_axes():
            return

        # TODO: 获取复位后的原始数据
        if self.version_int == 300:
            samples = self.md_dist._sample_async()
            if int(samples["freq"]) < self.md_dist.trigger_freq:
                logging.error(
                    "Post-homing adjustment measured samples %s(%s)",
                    samples, self.md_dist.trigger_freq
                )
                raise self.md_dist.printer.command_error(
                    "Toolhead stopped below model range"
                )
            dist = self.get_position_endstop()
        else:
            # After homing Z we perform a measurement and 
            # adjust the toolhead kinematic position.
            (dist, samples) = self.md_dist._sample(
                self.md_dist.z_settling_time, 10
            )
            if math.isinf(dist):
                logging.error(
                    "Post-homing adjustment measured samples %s",
                    samples
                )
                raise self.md_dist.printer.command_error(
                    "Toolhead stopped below model range"
                )
        homing_state.set_homed_position([None, None, dist])

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        stepper_mcu = stepper.get_mcu()
        trsync = trsyncs.get(stepper_mcu)
        if trsync is None:
            trsync = MCU_trsync(stepper_mcu, self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails, duplicated
        # from MCU_endstop
        sname = stepper.get_name()
        if sname.startswith("stepper_"):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")

    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]

    def home_start(self, print_time, sample_time,
                   sample_count, rest_time, triggered=True):
        if (self.md_dist.model is None) and (self.version_int != 300):
            raise self.md_dist.printer.command_error("No md_dist model loaded")

        self.is_homing = True
        # TODO: 新增对md_dist 3.0的支持, 主要是获取当前的有效数据,
        # 然后规划后发给下位机, 知道触发阈值.
        freq = None
        if self.version_int == 300:
            # samples = []
            # def cb(sample):
            #     samples.append(sample)

            # self.md_dist._sample_printtime_sync(50)
            # with self.md_dist.streaming_session(cb) as ss:
            #     # self.toolhead.dwell(0.250)
            #     self.md_dist._sample_printtime_sync(50)
            #     self.toolhead.dwell(0.01)
            # freqs = [s["freq"] for s in samples]
            # freqs_len = len(freqs)
            # if freqs_len == 0:
            #     raise self.md_dist.printer.command_error(
            #         "md_dist error: No data in samples")
            # freq = int(sum(freqs) / freqs_len) + self.md_dist.trigger_threshold
            self.toolhead.dwell(0.1)
            # freq = (int(self.md_dist._sample_async()["freq"]) +
            #         self.md_dist.trigger_threshold)
            samples = self.md_dist._sample_printtime_sync(10, 10)
            median_freq = median([s["freq"] for s in samples])
            freq = (int(median_freq) + self.md_dist.trigger_threshold)
            self.md_dist._apply_threshold(freq=freq) # 发送复位触发数据
        else:
            self.md_dist._apply_threshold(freq=freq) # 发送复位触发数据
            self.md_dist._sample_async()
        print_time = self.toolhead.get_last_move_time()
        clock = self._mcu.print_time_to_clock(print_time)
        rest_ticks = self._mcu.print_time_to_clock(print_time+rest_time)-clock
        self._rest_ticks = rest_ticks
        reactor = self._mcu.get_printer().get_reactor()
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for i, trsync in enumerate(self._trsyncs):
            try:
                trsync.start(
                    print_time, self._trigger_completion, expire_timeout
                )
            except TypeError:
                offset = float(i) / len(self._trsyncs)
                trsync.start(
                    print_time, offset, self._trigger_completion,
                    expire_timeout
                )
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        self.md_dist.md_dist_home_cmd.send([
            etrsync.get_oid(),
            etrsync.REASON_ENDSTOP_HIT,
            0,
        ])
        return self._trigger_completion

    def home_wait(self, home_end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        self.md_dist.md_dist_stop_home_cmd.send()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.0
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.0
        if self._mcu.is_fileoutput():
            return home_end_time
        return home_end_time

    def query_endstop(self, print_time):
        if self.md_dist.model is None:
            return 1
        clock = self._mcu.print_time_to_clock(print_time)
        sample = self.md_dist._sample_async()
        if self.md_dist.trigger_freq <= sample["freq"]:
            return 1
        else:
            return 0

    def get_position_endstop(self):
        return self.md_dist.trigger_distance

    def probing_move(self, pos, speed):
        # Perform probing move
        phoming = self.md_dist.printer.lookup_object('homing')
        epos = phoming.probing_move(self, pos, speed)
        return epos

class MD_Dist_Mesh_Helper:
    @classmethod
    def create(cls, md_dist, config):
        if config.has_section("bed_mesh"):
            return MD_Dist_Mesh_Helper(md_dist, config)
        else:
            return None

    def __init__(self, md_dist, config):
        self.md_dist = md_dist
        self.version_int = self.md_dist.version_int
        mesh_config = self.mesh_config = config.getsection("bed_mesh")
        self.bm = self.md_dist.printer.load_object(mesh_config, "bed_mesh")

        self.def_move_speed = mesh_config.getfloat(
            "speed", 50.0, above=0.0, note_valid=False
        )
        self.def_min_x, self.def_min_y = mesh_config.getfloatlist(
            "mesh_min", count=2, note_valid=False
        )
        self.def_max_x, self.def_max_y = mesh_config.getfloatlist(
            "mesh_max", count=2, note_valid=False
        )
        self.def_res_x, self.def_res_y = mesh_config.getintlist(
            "probe_count", count=2, note_valid=False
        )
        self.rri = mesh_config.getint(
            "relative_reference_index", None, note_valid=False
        )
        self.dir = config.getchoice(
            "mesh_main_direction",
            {'x':'x', 'X':'x', 'y':'y', 'Y':'y'},
            'y'
        )
        if self.def_max_x <= self.def_min_x:
            raise config.error("mesh_max_x must be greater than mesh_min_x")
        if self.def_max_y <= self.def_min_y:
            raise config.error("mesh_max_y must be greater than mesh_min_y")
        self.horizontal_move_z = mesh_config.getfloat('horizontal_move_z', 5.)
        self.overscan = config.getfloat("mesh_overscan", -1, minval=0)
        self.cluster_size = config.getfloat("mesh_cluster_size", 1, minval=0)
        self.runs = config.getint("mesh_runs", 1, minval=1)
        self.rewrite_command = config.getboolean('rewrite_command', True)
        self.def_probe_move = self.md_dist.speed

        # new line
        self.faulty_regions = []
        self.split_bed = config.getboolean('split_bed', False)
        self.split_bed_count = 0
        self.def_bed_maps = {'x':[], 'y':[],}
        self.bed_maps = {'x':[], 'y':[],}
        self.split_bed_pos = []
        self.split_bed_pos_count = []
        self.split_bed_pos_step = []
        self.split_pos_dict = {}
        x_start, x_end = (self.def_min_x, self.def_max_x)
        y_start, y_end = (self.def_min_y, self.def_max_y)
        x_step = (x_end - x_start) / (float)(self.def_res_x - 1)
        y_step = (y_end - y_start) / (float)(self.def_res_y - 1)
        if self.split_bed:
            if self.dir == 'x':
                for i in list(range(1, 100, 1)):
                    # if i == 1:
                    #     spos = config.getfloatlist(
                    #         "split_pos_%d" % (i,), count=2
                    #     )
                    # else:
                    spos = config.getfloatlist(
                        "split_pos_%d" % (i,), None, count=2
                    )
                    if spos is None:
                        break
                    if spos[1] <= spos[0]:
                        raise config.error("split_pos_%d data error: %.3f<=%.3f"
                                           % (i, spos[0], spos[1]))
                    if spos[0] < self.def_min_y or spos[1] > self.def_max_y:
                        raise config.error("split_pos_%d y out of range" % (i,))
                    spos_count = config.getint(
                        "split_pos_%d_count" % (i,), minval=2
                    )
                    y_end = spos[0]
                    self.split_bed_pos.append((x_start, x_end, y_start, y_end))
                    self.split_bed_pos_count.append(spos_count)
                    self.split_bed_pos_step.append(
                        (y_end - y_start) / (float)(spos_count - 1)
                    )
                    y_start = spos[1]
                    self.split_bed_count += 1
                y_end = self.def_max_y
                self.split_bed_pos.append((x_start, x_end, y_start, y_end))
                if sum(self.split_bed_pos_count) < self.def_res_y:
                    self.split_bed_pos_count.append(
                        (self.def_res_y - spos_count))
                    self.split_bed_pos_step.append(
                        (y_end - y_start) / (float)(spos_count - 1)
                    )
                elif sum(self.split_bed_pos_count) > self.def_res_y:
                    raise config.error("split_pos_count sum error")
                
                self.def_bed_maps['x'] = [x_start + i * x_step
                                          for i in range(self.def_res_x)]
                for i in range(0, len(self.split_bed_pos)):
                    split_len = self.split_bed_pos_count[i]
                    split_s, split_e = (self.split_bed_pos[i][2],
                                        self.split_bed_pos[i][3])
                    split_step = (split_e - split_s) / (float)(split_len - 1)
                    self.def_bed_maps['y'] += [split_s + j * split_step
                                                for j in range(split_len)]
            else:
                for i in list(range(1, 100, 1)):
                    spos = config.getfloatlist(
                        "split_pos_%d" % (i,), None, count=2
                    )
                    if spos is None:
                        break
                    if spos[1] <= spos[0]:
                        raise config.error("split_pos_%d data error: %.3f<=%.3f"
                                           % (i, spos[0], spos[1]))
                    if spos[0] < self.def_min_x or spos[1] > self.def_max_x:
                        raise config.error("split_pos_%d x out of range" % (i,))
                    spos_count = config.getint(
                        "split_pos_%d_count" % (i,)
                    )
                    x_end = spos[0]
                    self.split_bed_pos.append((x_start, x_end, y_start, y_end))
                    self.split_bed_pos_count.append(spos_count)
                    self.split_bed_pos_step.append(
                        (x_end - x_start) / (float)(spos_count - 1)
                    )
                    x_start = spos[1]
                    self.split_bed_count += 1
                x_end = self.def_max_x
                self.split_bed_pos.append((x_start, x_end, y_start, y_end))
                if sum(self.split_bed_pos_count) < self.def_res_x:
                    self.split_bed_pos_count.append(
                        (self.def_res_x - spos_count))
                    self.split_bed_pos_step.append(
                        (x_end - x_start) / (float)(spos_count - 1)
                    )
                elif sum(self.split_bed_pos_count) > self.def_res_x:
                    raise config.error("split_pos_count sum error")

                self.def_bed_maps['y'] = [y_start + i * y_step
                                          for i in range(self.def_res_y)]
                for i in range(0, len(self.split_bed_pos)):
                    split_len = self.split_bed_pos_count[i]
                    split_s, split_e = (self.split_bed_pos[i][0],
                                        self.split_bed_pos[i][1])
                    split_step = (split_e - split_s) / (float)(split_len - 1)
                    self.def_bed_maps['x'] += [split_s + j * split_step
                                                for j in range(split_len)]
        else:
            for i in list(range(1, 100, 1)):
                start = mesh_config.getfloatlist(
                    "faulty_region_%d_min" % (i,), None, count=2
                )
                if start is None:
                    break
                end = mesh_config.getfloatlist(
                    "faulty_region_%d_max" % (i,), count=2
                )
                x_min = min(start[0], end[0])
                x_max = max(start[0], end[0])
                y_min = min(start[1], end[1])
                y_max = max(start[1], end[1])
                self.faulty_regions.append(Region(x_min, x_max, y_min, y_max))
            

        self.gcode = self.md_dist.printer.lookup_object("gcode")
        if self.rewrite_command:
            self.prev_gcmd = self.gcode.register_command(
                "BED_MESH_CALIBRATE", None
            )
            self.gcode.register_command(
                "BED_MESH_CALIBRATE",
                self.cmd_BED_MESH_CALIBRATE,
                desc=self.cmd_BED_MESH_CALIBRATE_help
            )

        if self.overscan < 0:
            printer = self.md_dist.printer
            printer.register_event_handler(
                "klippy:mcu_identify",
                self._handle_mcu_identify
            )

        self.gcode.register_command(
            "MD_DIST_FLUSH_MAP",
            self.cmd_MD_DIST_FLUSH_MAP,
            desc=self.cmd_MD_DIST_FLUSH_MAP_help
        )

    cmd_BED_MESH_CALIBRATE_help = "Perform Mesh Bed Leveling"
    def cmd_BED_MESH_CALIBRATE(self, gcmd):
        method = gcmd.get("METHOD", "md_dist").lower()
        self._get_mesh_points(gcmd)
        logging.info(("BedMeshC:%s!" % (probe_object_name,)))
        if not self.profile_name.strip():
            raise gcmd.error("Value for parameter 'PROFILE' must be specified")
        self.bm.set_mesh(None)
        try:
            if method == "md_dist":
                if self.version_int == 300:
                    self.calibrate_v3(gcmd)
                else:
                    self.calibrate(gcmd)
            # else:
            #     self.prev_gcmd(gcmd)
        except bed_mesh.BedMeshError as e:
            raise gcmd.error(str(e))
        logging.info("MD_DIST_CALIBRATE done!"
                     f"{probe_object_name}: {self.profile_name}")

    def _handle_mcu_identify(self):
        # Auto determine a safe overscan amount
        toolhead = self.md_dist.printer.lookup_object("toolhead")
        curtime = self.md_dist.reactor.monotonic()
        status = toolhead.get_kinematics().get_status(curtime)
        xo = self.md_dist.x_offset
        yo = self.md_dist.y_offset
        settings = {
            'x': {
                "range": [self.def_min_x-xo,\
                          self.def_max_x-xo],
                "machine": [status["axis_minimum"][0],\
                            status["axis_maximum"][0]],
                "count": self.def_res_y,
            },
            'y': {
                "range": [self.def_min_y-yo,\
                          self.def_max_y-yo],
                "machine": [status["axis_minimum"][1],\
                            status["axis_maximum"][1]],
                "count": self.def_res_x,
            }
        }[self.dir]

        r = settings["range"]
        m = settings["machine"]
        space = (r[1] - r[0]) / (float(settings["count"]-1))
        self.overscan = min([
            max(0, r[0]-m[0]),
            max(0, m[1]-r[1]),
            space+2.0, # A half circle with 2mm lead in/out
        ])

    cmd_MD_DIST_FLUSH_MAP_help = "Flush Mesh Map"
    def cmd_MD_DIST_FLUSH_MAP(self, gcmd):
        self._get_mesh_points(gcmd)
        self._generate_path_split()

    # TODO: 获取调平数据, 计算调平点
    def _generate_path_v3(self):
        xo = self.md_dist.x_offset
        yo = self.md_dist.y_offset
        swap_coord = True if self.dir=='y' else False
        begin_x, end_x = [self.min_x-xo, self.max_x-xo]
        begin_y, end_y = [self.min_y-yo, self.max_y-yo]
        step_x = float((end_x - begin_x) / (float(self.res_x-1)))
        step_y = float((end_y - begin_y) / (float(self.res_y-1)))
        points = []
        pos_x = begin_x
        pos_y = begin_y
        if swap_coord:
            for i in range(0, self.res_x):
                for j in range(0, self.res_y):
                    points.append((pos_x, pos_y))
                    pos_y += step_y
                pos_x += step_x
                step_y = -step_y
                pos_y += step_y
        else:
            for i in range(0, self.res_y):
                for j in range(0, self.res_x):
                    points.append((pos_x, pos_y))
                    pos_x += step_x
                pos_y += step_y
                step_x = -step_x
                pos_x += step_x
        logging.info(f"\"{self.profile_name}\" points: {points}")
        return points

    def calibrate_v3(self, gcmd):
        toolhead = self.md_dist.printer.lookup_object("toolhead")
        # get params
        path = self._generate_path_v3()
        # path_len = len(path)
        probe_speed = gcmd.get_float(
            "PROBE_SPEED",
            self.def_probe_move, above=0.0
        )
        move_speed = gcmd.get_float("SPEED", self.def_move_speed, above=0.0)
        xo = self.md_dist.x_offset
        yo = self.md_dist.y_offset
        min_x = min(self.min_x, self.max_x)
        min_y = min(self.min_y, self.max_y)
        step_x = abs(self.step_x)
        step_y = abs(self.step_y)
        pos_xo = xo-min_x
        pos_yo = yo-min_y
        clusters = {}
        try:
            self.md_dist._start_streaming()
            for pos in path:
                # move z
                toolhead.manual_move([None, None, self.horizontal_move_z],
                                          probe_speed)
                toolhead.wait_moves()
                # move x,y
                toolhead.manual_move([pos[0], pos[1], None], move_speed)
                toolhead.wait_moves()
                # start probing
                epos = self.md_dist.run_probe(gcmd)
                gcmd.respond_info("Result is z=%.6f" % (epos[2],))
                index = (int((pos[0]+pos_xo)/step_x),
                         int((pos[1]+pos_yo)/step_y))
                clusters.update({index: epos[2]})
            logging.info(f"calibrate_v3: {clusters}")
            self._apply_mesh(clusters, gcmd)
        finally:
            self.md_dist._stop_streaming()
        
        toolhead.manual_move([None, None, self.horizontal_move_z],
                                    probe_speed)

    # 将bed_line填充到3个点
    def _post_process_bed_line(self, original_line, bed_line, index_s, index_e):
        if len(original_line) < 3:
            raise self.gcode.error("original line is too short")
        len_min = 0
        len_max = len(original_line)-1
        even = True
        line_temp = bed_line.copy()
        while(len(line_temp) < 3):
            if even:
                index_e += 1
                if index_e <= len_max:
                    line_temp.append(original_line[index_e]) # 加到末尾
            else:
                index_s -= 1
                if index_s >= len_min:
                    line_temp[:0] = [original_line[index_s]] # 加到开头
            
            even = not even
        bed_line[:] = line_temp

    def _generate_path_split(self):
        if not self.split_bed:
            return []

        xo = self.md_dist.x_offset
        yo = self.md_dist.y_offset

        swap_coord = False if self.dir == 'x' else True
        vertical_dir = 'y' if self.dir == 'x' else 'x'
        bed_line = []
        points = []
        self.split_pos_dict.clear()

        logging.info("bed_mesh s: (%.3f, %.3f)(%.3f, %.3f)" %
                     (self.min_x, self.min_y, self.max_x, self.max_y,))

        # 处理x方向
        bed_line.clear()
        the_last_pos = self.def_bed_maps['x'][0]
        the_index = the_index_s = the_index_e = 0
        for the_pos_x in self.def_bed_maps['x']:
            if self.min_x < the_pos_x:
                if len(bed_line) == 0:
                    the_index_s = the_index
                bed_line.append(the_last_pos)
            if ((self.max_x < the_pos_x) or 
                (the_pos_x == self.def_bed_maps['x'][-1])):
                if len(bed_line) == 0:
                    raise self.gcode.error("No bed line found")
                if bed_line[0] != the_pos_x:
                    bed_line.append(the_pos_x)
                the_index_e = the_index
                break
            the_last_pos = the_pos_x
            the_index += 1
        # 后处理, 确保扫描点个数大于3
        if len(bed_line) < 3:
            self._post_process_bed_line(self.def_bed_maps['x'], bed_line,
                                        the_index_s, the_index_e)
        self.bed_maps['x'][:] = [num - xo for num in bed_line]
        # 重构数据,方便后续计算
        self.min_x, self.max_x = (bed_line[0], bed_line[-1])
        self.res_x = len(bed_line)
        self.step_x = (self.max_x - self.min_x) / (self.res_x - 1)
        

        # 处理y方向
        bed_line.clear()
        the_last_pos = self.def_bed_maps['y'][0]
        the_index = the_index_s = the_index_e = 0
        for the_pos_y in self.def_bed_maps['y']:
            if self.min_y < the_pos_y:
                if len(bed_line) == 0:
                    the_index_s = the_index
                bed_line.append(the_last_pos)
            if ((self.max_y < the_pos_y) or
                (the_pos_y == self.def_bed_maps['y'][-1])):
                if len(bed_line) == 0:
                    raise self.gcode.error("No bed line found")
                if bed_line[0] != the_pos_y:
                    bed_line.append(the_pos_y)
                the_index_e = the_index
                break
            the_last_pos = the_pos_y
            the_index += 1
        # 后处理, 确保扫描点个数大于3
        if len(bed_line) < 3:
            self._post_process_bed_line(self.def_bed_maps['y'], bed_line,
                                        the_index_s, the_index_e)
        self.bed_maps['y'][:] = [num - yo for num in bed_line]
        # 重构数据,方便后续计算
        self.min_y, self.max_y = (bed_line[0], bed_line[-1])
        self.res_y = len(bed_line)
        self.step_y = (self.max_y - self.min_y) / (self.res_y - 1)

        for i in range(len(self.bed_maps[vertical_dir])):
            self.split_pos_dict[int(self.bed_maps[vertical_dir][i])] = i

        logging.info("bed_map: %s", self.bed_maps)
        logging.info("def_bed_map: %s", self.def_bed_maps)

        logging.info("bed_mesh e: (%.3f, %.3f)(%.3f, %.3f)" %
                     (self.min_x, self.min_y, self.max_x, self.max_y,))

        # 画弧线
        points.clear()
        begin_a = self.bed_maps[self.dir][0]
        end_a = self.bed_maps[self.dir][-1]
        pos_last = self.bed_maps[vertical_dir][0]
        the_index = 0
        for pos_end in self.bed_maps[vertical_dir]:
            even = the_index % 2 == 0
            step = (pos_end - pos_last)
            corner_radius = min(step/2.0, self.overscan)
            pa = (begin_a, pos_end) if even else (end_a, pos_end)
            pb = (end_a, pos_end) if even else (begin_a, pos_end)
            if (the_index > 0) and (corner_radius > 0):
                if even:
                    center = begin_a - self.overscan + corner_radius
                    points += arc_points(
                        center, pos_end - step + corner_radius,
                        corner_radius, -90, -90
                    )
                    points += arc_points(
                        center, pos_end - corner_radius,
                        corner_radius, -180, -90
                    )
                else:
                    center = end_a + self.overscan - corner_radius
                    points += arc_points(
                        center, pos_end - step + corner_radius,
                        corner_radius, -90, 90
                    )
                    points += arc_points(
                        center, pos_end - corner_radius,
                        corner_radius, 0, 90
                    )
            points.append(pa)
            points.append(pb)
            pos_last = pos_end
            the_index += 1

        if swap_coord:
            for i in range(len(points)):
                (x,y) = points[i]
                points[i] = (y,x)

        return points

    def _generate_path(self):
        xo = self.md_dist.x_offset
        yo = self.md_dist.y_offset
        settings = {
            'x': {
                "range_aligned": [self.min_x-xo, self.max_x-xo],
                "range_perpendicular": [self.min_y-yo, self.max_y-yo],
                "count": self.res_y,
                "swap_coord": False,
            },
            'y': {
                "range_aligned": [self.min_y-yo, self.max_y-yo],
                "range_perpendicular": [self.min_x-xo, self.max_x-xo],
                "count": self.res_x,
                "swap_coord": True,
            }
        }[self.dir]

        # We build the path in "normalized" coordinates and then simply
        # swap x and y at the end if we need to
        begin_a, end_a = settings["range_aligned"]
        begin_p, end_p = settings["range_perpendicular"]
        swap_coord = settings["swap_coord"]
        step = (end_p - begin_p) / (float(settings["count"]-1))
        points = []
        corner_radius = min(step/2, self.overscan)
        for i in range(0, settings["count"]):
            pos_p = begin_p + step * i
            even = i % 2 == 0 # If even we are going 'right', else 'left'
            pa = (begin_a, pos_p) if even else (end_a, pos_p)
            pb = (end_a, pos_p) if even else (begin_a, pos_p)

            l = (pa,pb)

            if len(points) > 0 and corner_radius > 0:
                # We need to insert an overscan corner. Basically we insert
                # a rounded rectangle to smooth out the transition and retain
                # as much speed as we can.
                #
                #  ---|---<
                # /
                # |
                # \
                #  ---|--->
                #
                # We just need to draw the two 90 degree arcs. They contain
                # the endpoints of the lines connecting everything.
                if even:
                    center = begin_a - self.overscan + corner_radius
                    points += arc_points(
                        center, pos_p - step + corner_radius,
                        corner_radius, -90, -90
                    )
                    points += arc_points(
                        center, pos_p - corner_radius,
                        corner_radius, -180, -90
                    )
                else:
                    center = end_a + self.overscan - corner_radius
                    points += arc_points(
                        center, pos_p - step + corner_radius,
                        corner_radius, -90, 90
                    )
                    points += arc_points(
                        center, pos_p - corner_radius,
                        corner_radius, 0, 90
                    )

            points.append(l[0])
            points.append(l[1])

        if swap_coord:
            for i in range(len(points)):
                (x,y) = points[i]
                points[i] = (y,x)

        return points

    def calibrate(self, gcmd):
        self.toolhead = self.md_dist.toolhead
        if self.split_bed:
            path = self._generate_path_split()
        else:
            path = self._generate_path()

        probe_speed = gcmd.get_float(
            "PROBE_SPEED",
            self.md_dist.speed, above=0.0
        )
        self.md_dist._move_to_probing_height(probe_speed)

        speed = gcmd.get_float("SPEED", self.def_move_speed, above=0.0)
        runs = gcmd.get_int("RUNS", self.runs, minval=1)

        try:
            self.md_dist._start_streaming()

            # Move to first location
            (x,y) = path[0]
            self.toolhead.manual_move([x, y, None], speed)
            self.toolhead.wait_moves()

            self.md_dist._sample_printtime_sync(5)
            clusters = self._sample_mesh(gcmd, path, speed, runs)

        finally:
            self.md_dist._stop_streaming()

        clusters = self._interpolate_faulty(clusters)
        self._apply_mesh(clusters, gcmd)

        if ((self.md_dist.end_park_x is not None) and
            (self.md_dist.end_park_y is not None)):
            if (self.md_dist.first_move_axis == 'x'):
                self.toolhead.manual_move(
                    [self.md_dist.end_park_x, None, None],
                    speed
                )
                self.toolhead.wait_moves()
            else:
                self.toolhead.manual_move(
                    [None, self.md_dist.end_park_y, None],
                    speed
                )
                self.toolhead.wait_moves()

            self.toolhead.manual_move(
                [self.md_dist.end_park_x, self.md_dist.end_park_y, None],
                speed
            )
            self.toolhead.wait_moves()
    
    def _get_mesh_points(self, gcmd):
        self.profile_name = gcmd.get("PROFILE", "default")
        self.min_x, self.min_y = coord_fallback(
            gcmd, "MESH_MIN", float,
            self.def_min_x, self.def_min_y, lambda v, d: max(v, d)
        )
        self.max_x, self.max_y = coord_fallback(
            gcmd, "MESH_MAX", float,
            self.def_max_x, self.def_max_y, lambda v, d: min(v, d)
        )
        self.res_x, self.res_y = coord_fallback(
            gcmd, "PROBE_COUNT", int,
            self.def_res_x, self.def_res_y, lambda v, _d: max(v, 3)
        )
        if self.min_x > self.max_x:
            self.min_x, self.max_x = (
                max(self.max_x, self.def_min_x),
                min(self.min_x, self.def_max_x)
            )
        if self.min_y > self.max_y:
            self.min_y, self.max_y = (
                max(self.max_y, self.def_min_y),
                min(self.min_y, self.def_max_y)
            )

        self.step_x = (self.max_x - self.min_x) / (self.res_x - 1)
        self.step_y = (self.max_y - self.min_y) / (self.res_y - 1)

    def _fly_path(self, path, speed, runs):
        # Run through the path
        for i in range(runs):
            p = path if i % 2 == 0 else reversed(path)
            for (x,y) in p:
                self.toolhead.manual_move([x, y, None], speed)
        self.toolhead.wait_moves()

    def _is_valid_position(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.min_y

    # 根据提供的地址来获取网格点对应的样本距离
    def _sample_mesh(self, gcmd, path, speed, runs):
        cs = gcmd.get_float("CLUSTER_SIZE", self.cluster_size, minval=0.0)

        min_x, min_y = self.min_x, self.min_y
        xo, yo = self.md_dist.x_offset, self.md_dist.y_offset

        clusters = {}
        total_samples = [0]
        invalid_samples = [0]

        if self.split_bed_count:
            if self.dir == 'x':
                def cb(sample):
                    total_samples[0] += 1
                    d = sample["dist"]
                    (x, y, z) = sample["pos"]
                    x += xo
                    # y += yo

                    if math.isinf(d):
                        if self._is_valid_position(x, y+yo):
                            invalid_samples[0] += 1
                        return

                    # Calculate coordinate of the cluster we are in
                    xi = int(round((x - min_x) / self.step_x))
                    yi = self.split_pos_dict.get(int(y), -99)

                    k = (xi, yi)

                    if k not in clusters:
                        clusters[k] = []
                    clusters[k].append(d)
            else:
                def cb(sample):
                    total_samples[0] += 1
                    d = sample["dist"]
                    (x, y, z) = sample["pos"]
                    # x += xo
                    y += yo

                    if math.isinf(d):
                        if self._is_valid_position(x+xo, y):
                            invalid_samples[0] += 1
                        return

                    # Calculate coordinate of the cluster we are in
                    xi = self.split_pos_dict.get(int(x), -99)
                    yi = int(round((y - min_y) / self.step_y))

                    k = (xi, yi)

                    if k not in clusters:
                        clusters[k] = []
                    clusters[k].append(d)
        else:
            def cb(sample):
                total_samples[0] += 1
                d = sample["dist"]
                (x, y, z) = sample["pos"]
                x += xo
                y += yo

                if math.isinf(d):
                    if self._is_valid_position(x, y):
                        invalid_samples[0] += 1
                    return

                # Calculate coordinate of the cluster we are in
                xi = int(round((x - min_x) / self.step_x))
                yi = int(round((y - min_y) / self.step_y))

                # If there's a cluster size limit, apply it here
                if cs > 0:
                    xf = xi * self.step_x + min_x
                    yf = yi * self.step_y + min_y
                    dx = x - xf
                    dy = y - yf
                    dist = math.sqrt(dx*dx+dy*dy)
                    if dist > cs:
                        return

                k = (xi, yi)

                if k not in clusters:
                    clusters[k] = []
                clusters[k].append(d)

        with self.md_dist.streaming_session(cb) as ss:
            self._fly_path(path, speed, runs)

        gcmd.respond_info(
            "Sampled %d total points over %d runs" % (total_samples[0], runs)
        )
        if invalid_samples[0]:
            gcmd.respond_info(
                "!! Encountered %d invalid samples!" % (invalid_samples[0],)
            )
        gcmd.respond_info(
            "Samples binned in %d clusters" % (len(clusters),)
        )
        # logging.info("_sample_mesh: %s" % (clusters,))
        return clusters

    def _is_faulty_coordinate(self, x, y, add_offsets=False):
        if add_offsets:
            xo, yo = self.md_dist.x_offset, self.md_dist.y_offset
            x += xo
            y += yo
        for r in self.faulty_regions:
            if r.is_point_within(x, y):
                return True
        return False

    def _interpolate_faulty(self, clusters):
        faulty_indexes = []
        xi_max = 0
        yi_max = 0
        for (xi, yi), points in clusters.items():
            if xi > xi_max:
                xi_max = xi
            if yi > yi_max:
                yi_max = yi
            xc = xi * self.step_x + self.min_x
            yc = yi * self.step_y + self.min_y
            if self._is_faulty_coordinate(xc, yc):
                clusters[(xi, yi)] = None
                faulty_indexes.append((xi, yi))

        def get_nearest(start, dx, dy):
            (x, y) = start
            x += dx
            y += dy
            while (x >= 0 and x <= xi_max and y >= 0 and y <= yi_max):
                if clusters[(x, y)] is not None:
                    return (abs(x-start[0])+abs(y-start[0]), \
                            median(clusters[(x,y)]))
                x += dx
                y += dy
            return None

        def interp_weighted(lower, higher):
            if lower is None and higher is None:
                return None
            if lower is None and higher is not None:
                return higher[1]
            elif lower is not None and higher is None:
                return lower[1]
            else:
                return ((lower[1] * lower[0] + higher[1] * higher[0]) \
                        / (lower[0] + higher[0]))

        for coord in faulty_indexes:
            xl = get_nearest(coord, -1,  0)
            xh = get_nearest(coord,  1,  0)
            xavg = interp_weighted(xl, xh)
            yl = get_nearest(coord,  0, -1)
            yh = get_nearest(coord,  0,  1)
            yavg = interp_weighted(yl, yh)
            avg = None
            if xavg is not None and yavg is None:
                avg = xavg
            elif xavg is None and yavg is not None:
                avg = yavg
            else:
                avg = (xavg + yavg) / 2.0
            clusters[coord] = [avg]

        return clusters

    def _apply_mesh(self, clusters, gcmd):
        matrix = []
        td = self.md_dist.trigger_distance
        for yi in range(self.res_y):
            line = []
            if self.version_int == 300:
                for xi in range(self.res_x):
                    cluster = clusters.get((xi,yi), None)
                    if cluster is None:
                        xc = xi * self.step_x + self.min_x
                        yc = yi * self.step_y + self.min_y
                        logging.info(
                            "Cluster (%.3f,%.3f)[%d,%d] is empty!"
                            % (xc, yc, xi, yi)
                        )
                        err = (
                            "Empty clusters found"
                        )
                        raise self.gcode.error(err)
                    line.append(cluster)
            else:
                for xi in range(self.res_x):
                    cluster = clusters.get((xi,yi), None)
                    if cluster is None or len(cluster) == 0:
                        xc = xi * self.step_x + self.min_x
                        yc = yi * self.step_y + self.min_y
                        logging.info(
                            "Cluster (%.3f,%.3f)[%d,%d] is empty!"
                            % (xc, yc, xi, yi)
                        )
                        err = (
                            "Empty clusters found\n"
                            "Try increasing mesh cluster_size or slowing down"
                        )
                        raise self.gcode.error(err)
                    data = [td-d for d in cluster]
                    line.append(median(data))
            matrix.append(line)

        rri = gcmd.get_int("RELATIVE_REFERENCE_INDEX", self.rri)
        if rri is not None:
            if rri < 0 or rri >= self.res_x * self.res_y:
                rri = None

        if rri is not None:
            rri_x = rri % self.res_x
            rri_y = int(math.floor(rri / self.res_x))
            z_offset = matrix[rri_y][rri_x]
            for i, line in enumerate(matrix):
                matrix[i] = [z-z_offset for z in line]

        params = dict(self.bm.bmc.mesh_config)
        params["min_x"] = self.min_x
        params["max_x"] = self.max_x
        params["min_y"] = self.min_y
        params["max_y"] = self.max_y
        params["x_count"] = self.res_x
        params["y_count"] = self.res_y
        try:
            mesh = bed_mesh.ZMesh(params)
        except:
            mesh = bed_mesh.ZMesh(params, self.profile_name)
        try:
            mesh.build_mesh(matrix)
        except bed_mesh.BedMeshError as e:
            raise self.gcode.error(str(e))
        self.bm.set_mesh(mesh)
        self.gcode.respond_info("Mesh calibration complete")
        self.bm.save_profile(self.profile_name)

class Region:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def is_point_within(self, x, y):
        return ((x > self.x_min and x < self.x_max) \
                and (y > self.y_min and y < self.y_max))

def arc_points(cx, cy, r, start_angle, span):
    # Angle delta is determined by a max deviation(md) from 0.1mm:
    #   r * versin(d_a) < md
    #   versin(d_a) < md/r
    #   d_a < arcversin(md/r)
    #   d_a < arccos(1-md/r)
    # We then determine how many of these we can fit in exactly
    # 90 degrees(rounding up) and then determining the exact
    # delta angle.
    start_angle = start_angle / 180.0 * math.pi
    span = span / 180.0 * math.pi
    d_a = math.acos(1 - 0.1 / r)
    cnt = int(math.ceil(abs(span) / d_a))
    d_a = span / float(cnt)

    points = []
    for i in range(cnt+1):
        ang = start_angle + d_a*float(i)
        x = cx + math.cos(ang)*r
        y = cy + math.sin(ang)*r
        points.append((x,y))

    return points

def coord_fallback(gcmd, name, parse, def_x, def_y, map=lambda v, d: v):
    param = gcmd.get(name, None)
    if param is not None:
        try:
            x, y = [parse(p.strip()) for p in param.split(",", 1)]
            return map(x, def_x), map(y, def_y)
        except:
            raise gcmd.error("Unable to parse parameter '%s'" % (name,))
    else:
        return def_x, def_y

def median(samples):
    return float(np.median(samples))

def load_config(config):
    md_dist = MD_Dist_Probe(config)
    config.get_printer().add_object(
        probe_object_name, MD_Dist_Probe_Wrapper(md_dist))
    if md_dist.enable_temp_compensate:
        temp = MD_Dist_Temp_Wrapper(md_dist)
        config.get_printer().add_object("temperature_sensor MD_DIST", temp)
        pheaters = md_dist.printer.load_object(config, "heaters")
        pheaters.available_sensors.append("temperature_sensor MD_DIST")
    return md_dist

def load_config_prefix(config):
    md_dist = config.get_printer().lookup_object("md_dist")
    name = config.get_name()
    if name.startswith("md_dist model "):
        name = name[14:]
        model = MD_Dist_Model.load(name, config, md_dist)
        md_dist._register_model(name, model)
        return model
    else:
        raise config.error("Unknown MD_DIST config directive '%s'"
              % (name[7:],))
