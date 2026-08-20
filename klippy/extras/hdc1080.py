# Klipper module for reading temperature and humidity from Linux IIO devices
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os, logging, time
from . import bus

class HDC1080:
    HDC1080_ADDR            = 0x40
    HDC1080_TEMPERATURE     = 0x00
    HDC1080_HUMIDITY        = 0x01
    HDC1080_CONFIGURATION   = 0x02
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]

        # 更新间隔
        self.update_interval = config.getfloat('update_interval', 2.0,
                                               minval=0.5)

        # IIO设备路径配置
        self.use_pi_module = True
        # default: '/sys/bus/iio/devices/iio:device0/in_temp_raw'
        self.temp_path = config.get('temp_path', None)
        # default: '/sys/bus/iio/devices/iio:device0/in_humidityrelative_raw'
        self.humidity_path = config.get('humidity_path', None)
        if self.temp_path is None or self.humidity_path is None:
            # self.i2c = bus.MCU_I2C_from_config(config,
            #                             default_addr=self.HDC1080_ADDR,
            #                             default_speed=400000)
            # self.mcu = mcu = self.i2c.get_mcu()
            # Chip options
            sda_pin_name = config.get('sda_pin')
            scl_pin_name = config.get('scl_pin')
            ppins = self.printer.lookup_object('pins')
            sda_ppin = ppins.lookup_pin(sda_pin_name)
            scl_ppin = ppins.lookup_pin(scl_pin_name)
            self.mcu = mcu = sda_ppin['chip']
            if scl_ppin['chip'] is not mcu:
                raise config.error("%s config error: All pins must be "
                                "connected to the same MCU" % (self.name,))
            sda_pin = sda_ppin['pin']
            scl_pin = scl_ppin['pin']
            speed = config.getint('i2c_speed', 100000, minval=100000)
            i2c_ticks = mcu.seconds_to_clock(1. / speed) # 当前为0
            self.wait_time = config.getfloat('wait_time', 0.02,
                                        minval=0.02, maxval=0.2)
            self.wait_ticks = 0
            self.rest_ticks = 0
            self.cmd_queue = self.mcu.alloc_command_queue()
            # self.i2c_oid = mcu.create_oid()
            # mcu.add_config_cmd("config_i2c oid=%d" % (self.i2c_oid,))
            self.oid = mcu.create_oid()
            logging.info(
                "config_hdc1080 oid=%d scl_pin=%s sda_pin=%s"\
                " ticks=%u address=%d"
                % (self.oid, scl_pin, sda_pin,
                   i2c_ticks, self.HDC1080_ADDR))
            mcu.add_config_cmd(
                "config_hdc1080 oid=%d scl_pin=%s sda_pin=%s"\
                " ticks=%u address=%d"
                % (self.oid, scl_pin, sda_pin,
                   i2c_ticks, self.HDC1080_ADDR))
            
            # mcu.add_config_cmd("hdc1080_start oid=%c reg=%u read_len=%u"\
            #     " rest_ticks=%u wait_ticks=%u"
            #     % (self.oid, self.HDC1080_TEMPERATURE, 4,
            #        0, 0,))
            self.use_pi_module = False

        # 转换系数
        self.temp_scale = config.getfloat('temp_scale', 165.0)
        self.temp_offset = config.getfloat('temp_offset', -40.0)
        self.humidity_scale = config.getfloat('humidity_scale', 100.0)
        self.sensor_min_temp = config.getfloat('sensor_min_temp', None)
        self.sensor_max_temp = config.getfloat('sensor_max_temp', None)
        
        # 内部状态
        self.temperature = 0.0
        self.humidity = 0.0
        self.last_update = 0
        self.min_temp = self.max_temp = 0.0
        self._callback_th = self._callback = None
        
        self.reactor = self.printer.get_reactor()
        # 注册事件
        if self.use_pi_module:
            self.printer.register_event_handler("klippy:ready",
                                                self._handle_ready)
        else:
            self.hdc1080_read_cmd = self.hdc1080_reg_write = None
            self.mcu.register_response(
                self._hdc1080_response,
                "hdc1080_data", self.oid,
            )
            self.mcu.register_config_callback(self._build_config)
            self.printer.register_event_handler("klippy:ready",
                                                self._handle_ready)
            # self.printer.register_event_handler("klippy:mcu_identify",
            #                                     self._handle_connect)

        # 注册命令
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('QUERY_DRYER_SENSOR', 
                              self.cmd_QUERY_DRYER_SENSOR,
                              desc=self.cmd_QUERY_DRYER_SENSOR_help)
        
        # 注册到温度传感器系统
        self.printer.add_object("hdc1080 " + self.name, self)

    def _handle_connect(self):
        logging.info(f"Connecting to {self.name}")
        if not self.use_pi_module:
            self._init_hdc1080()
            # self.reactor.register_timer(self._init_hdc1080,
            #     self.reactor.monotonic() + 5.0 )#self.update_interval)

    def _handle_ready(self):
        if self.use_pi_module:
            self.reactor.register_timer(self._update_sensor,
                                        self.reactor.NOW + self.update_interval)
        else:
            self._init_hdc1080()

    def _build_config(self):
        # logging.info("hdc1080 build_config")
        self.hdc1080_write_cmd = self.mcu.lookup_command(
            "hdc1080_reg_write oid=%c data=%*s", cq=self.cmd_queue)
        self.hdc1080_read_cmd = self.mcu.lookup_query_command(
            "hdc1080_reg_read oid=%c reg=%*s read_len=%u",
            "hdc1080_read_response oid=%c response=%*s", oid=self.oid,
            cq=self.cmd_queue)
        self.hdc1080_start_cmd = self.mcu.lookup_command(
            "hdc1080_start oid=%c reg=%u read_len=%u"\
            " rest_ticks=%u wait_ticks=%u",
            cq=self.cmd_queue)
        # self._init_hdc1080()

    def setup_minmax(self, min_temp, max_temp):
        if self.sensor_min_temp is not None:
            min_temp = min(min_temp, self.sensor_min_temp)
        if self.sensor_max_temp is not None:
            max_temp = max(max_temp, self.sensor_max_temp)
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def setup_callback_th(self, cb):
        self._callback_th = cb

    def _read_raw_value(self, path):
        """读取IIO设备原始值"""
        try:
            with open(path, 'r') as f:
                value = f.read().strip()
                return int(value)
        except (IOError, ValueError) as e:
            logging.warning(f"Failed to read from {path}: {e}")
            return None

    def _update_sensor(self, eventtime):
        """定期更新传感器数据"""
        # 读取温度原始值
        temp_raw = self._read_raw_value(self.temp_path)
        if temp_raw is not None:
            # 转换公式: temp = (raw / 65536) * 165 - 40
            self.temperature = ((temp_raw / 65536.0) * self.temp_scale +
                                self.temp_offset)

        # 读取湿度原始值
        humidity_raw = self._read_raw_value(self.humidity_path)
        if humidity_raw is not None:
            # 转换公式: humidity = (raw / 65536) * 100
            self.humidity = (humidity_raw / 65536.0) * self.humidity_scale
            # 限制在0-100范围
            self.humidity = max(0.0, min(100.0, self.humidity))

        self.last_update = eventtime
        # 发送更新事件
        self.printer.send_event("hdc1080_sensor:update", eventtime)
        # 返回下次更新时间
        return eventtime + self.update_interval

    # 初始化HDC1080传感器
    def _init_hdc1080(self):
    # def _init_hdc1080(self, eventtime):
        config_data = 0x1000
        msg = [self.HDC1080_CONFIGURATION, 0x10, 0x00]
        self.hdc1080_write_cmd.send([self.oid, msg])
        # Wait 15ms after reset
        self.reactor.pause(self.reactor.monotonic() + .010)
        # Check if the reset is complete
        msg = [self.HDC1080_CONFIGURATION]
        params = self.hdc1080_read_cmd.send([self.oid, msg, 2])
        response = bytearray(params['response'])
        get_data = response[0]<<8 | response[1]
        if get_data != config_data:
            msg = f"Failed to initialize {self.name}(HDC1080): {get_data}"
            logging.error(msg)
            raise Exception(msg)
        self.wait_ticks = self.mcu.seconds_to_clock(self.wait_time)
        self.rest_ticks = self.mcu.seconds_to_clock(self.update_interval)
        logging.info("hdc1080_start oid=%d reg=%u read_len=%u"\
            " rest_ticks=%u wait_ticks=%u"
            % (self.oid, self.HDC1080_TEMPERATURE, 4,
               self.rest_ticks, self.wait_ticks,))
        # self.mcu.add_config_cmd("hdc1080_start oid=%c reg=%u read_len=%u"\
        #     " rest_ticks=%u wait_ticks=%u"
        #     % (self.oid, self.HDC1080_TEMPERATURE, 4,
        #        self.rest_ticks, self.wait_ticks,))
        msg = [self.oid, self.HDC1080_TEMPERATURE, 4,
               self.rest_ticks, self.wait_ticks]
        self.hdc1080_start_cmd.send(msg)
        logging.info(f"Initialized {self.name}(hdc1080): {get_data}")

    # 更新HDC1080传感器数据
    def _hdc1080_response(self, params):
        """定期更新HDC1080传感器数据"""
        # logging.info("_hdc1080_response:%s" % (params,))
        raw_data = bytearray(params['response'])
        if raw_data is not None:
            temp_raw = float((raw_data[0] << 8) | raw_data[1])
            humidity_raw = float((raw_data[2] << 8) | raw_data[3])
            # 转换公式: temp = (raw/65536) * 165 - 40
            self.temperature = ((temp_raw*self.temp_scale/65536) + 
                                self.temp_offset)
            # 转换公式: humidity = (raw/65536) * 100
            self.humidity = (humidity_raw*self.humidity_scale/65536)
            # 确定范围
            if ((self.temperature < self.min_temp) or
                (self.temperature > self.max_temp)):
                self.printer.invoke_shutdown(
                    "HDC1080 temperature %0.1f outside range of %0.1f:%.01f"
                    % (self.temperature, self.min_temp, self.max_temp))
            self.humidity = max(0.0, min(100.0, self.humidity))
            self.last_update = self.reactor.monotonic()
            # logging.info("hdc1080 update(%.2f): T%.2f, H:%.2f" %
            #              (self.last_update, self.temperature, self.humidity,))
            printer_time = self.mcu.estimated_print_time(self.last_update)
            if self._callback is not None:
                self._callback(printer_time, self.temperature)
            elif self._callback_th is not None:
                self._callback_th(printer_time, self.temperature, self.humidity)
            self.printer.send_event("hdc1080_sensor:update", self.last_update)

    def get_status(self, eventtime=None):
        """获取传感器状态"""
        return {
            'last_update': round(self.last_update, 2),
            'temperature': round(self.temperature, 2),
            'humidity': round(self.humidity, 2),
        }
        
    def get_temp(self):
        """获取温度值（兼容性接口）"""
        return self.temperature
        
    def get_humidity(self):
        """获取湿度值"""
        return self.humidity
        
    cmd_QUERY_DRYER_SENSOR_help = "Query dryer sensor temperature and humidity"
    def cmd_QUERY_DRYER_SENSOR(self, gcmd):
        """查询命令处理"""
        gcmd.respond_info(
            f"Dryer Sensor [{self.name}]:\n"
            f"  Last Update: {self.last_update:.1f}s\n"
            f"  Temperature: {self.temperature:.1f} C\n"
            f"  Humidity: {self.humidity:.1f} %RH")

# def load_config_prefix(config):
#     return DryerSensor(config)

def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("HDC1080", HDC1080)
