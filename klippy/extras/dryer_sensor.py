# Klipper module for reading temperature and humidity from Linux IIO devices
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import logging
import time

class DryerSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[-1]
        
        # IIO设备路径配置
        self.temp_path = config.get('temp_path', 
                                    '/sys/bus/iio/devices/iio:device0/in_temp_raw')
        self.humidity_path = config.get('humidity_path', 
                                        '/sys/bus/iio/devices/iio:device0/in_humidityrelative_raw')
        
        # 更新间隔
        self.update_interval = config.getfloat('update_interval', 2.0, minval=0.5)
        
        # 转换系数
        self.temp_scale = config.getfloat('temp_scale', 165.0)
        self.temp_offset = config.getfloat('temp_offset', -40.0)
        self.humidity_scale = config.getfloat('humidity_scale', 100.0)
        
        # 内部状态
        self.temperature = 0.0
        self.humidity = 0.0
        self.last_update = 0
        
        # 注册事件
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        
        # 注册命令
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('QUERY_DRYER_SENSOR', 
                              self.cmd_QUERY_DRYER_SENSOR,
                              desc=self.cmd_QUERY_DRYER_SENSOR_help)
        
        # 注册到温度传感器系统
        self.printer.add_object("dryer_sensor " + self.name, self)
        
    def _handle_ready(self):
        self.reactor = self.printer.get_reactor()
        self.reactor.register_timer(self._update_sensor, self.reactor.NOW)
        
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
            self.temperature = (temp_raw / 65536.0) * self.temp_scale + self.temp_offset
            
        # 读取湿度原始值
        humidity_raw = self._read_raw_value(self.humidity_path)
        if humidity_raw is not None:
            # 转换公式: humidity = (raw / 65536) * 100
            self.humidity = (humidity_raw / 65536.0) * self.humidity_scale
            self.humidity = max(0.0, min(100.0, self.humidity))  # 限制在0-100范围
            
        self.last_update = eventtime
        
        # 发送更新事件
        self.printer.send_event("dryer_sensor:update", eventtime)
        
        # 返回下次更新时间
        return eventtime + self.update_interval
        
    def get_status(self, eventtime=None):
        """获取传感器状态"""
        return {
            'temperature': self.temperature,
            'humidity': self.humidity,
            'last_update': self.last_update
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
        gcmd.respond_info(f"Dryer Sensor [{self.name}]:\n"
                         f"  Temperature: {self.temperature:.1f} C\n"
                         f"  Humidity: {self.humidity:.1f} %RH\n"
                         f"  Last Update: {time.time() - self.last_update:.1f}s ago")

def load_config_prefix(config):
    return DryerSensor(config)