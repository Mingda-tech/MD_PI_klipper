# RFID sensor support for RC523/MFRC523 with 8-channel antenna switching
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import struct
import json
from . import bus

# OpenTag data format constants
OPENTAG_VERSION = 1
OPENTAG_HEADER_SIZE = 76
OPENTAG_OPTIONAL_SIZE = 96
OPENTAG_TOTAL_SIZE = 172

# RFID status codes
MI_OK = 0
MI_NOTAGERR = 1
MI_ERR = 2

class RFIDSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.bulk_tag_data_buffers = {}
        
        # SPI configuration (only initialize once!)
        self.spi = bus.MCU_SPI_from_config(config, 0, default_speed=1500000)
        self.mcu = self.spi.get_mcu()
        
        # Configuration
        self.scan_interval = config.getfloat('scan_interval', 1.0, minval=0.1)
        self.power_level = config.getint('power_level', 2, minval=0, maxval=3)
        self.auto_load = config.getboolean('auto_load', True)
        
        # Pin configuration for antenna channels
        ppins = self.printer.lookup_object('pins')
        self.antenna_pins = []
        for i in range(1, 9):
            pin_a = config.get(f'channel{i}_pin_a')
            pin_b = config.get(f'channel{i}_pin_b')
            # Store pin parameters for later use
            self.antenna_pins.append(ppins.lookup_pin(pin_a))
            self.antenna_pins.append(ppins.lookup_pin(pin_b))
        
        # State tracking
        self.tag_data = [None] * 8  # Store tag data for each channel
        self.tag_present = [False] * 8
        self.scanning = False
        self.current_channel = 0
        
        # MCU commands
        self.oid = self.mcu.create_oid()
        
        # Register commands
        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(self._handle_rfid_read_response,
                                   "rfid_read_response", self.oid)
        self.mcu.register_response(self._handle_rfid_data_response,
                                   "rfid_data_response", self.oid)
        self.mcu.register_response(self._handle_rfid_read_complete,
                                   "rfid_read_complete", self.oid)
        self.mcu.register_response(self._handle_rfid_status_response,
                                   "rfid_status_response", self.oid)
        self.mcu.register_response(self._handle_rfid_write_response,
                                   "rfid_write_response", self.oid)
        
        # G-code commands
        self.gcode.register_command('RFID_READ',
                                   self.cmd_RFID_READ,
                                   desc=self.cmd_RFID_READ_help)
        self.gcode.register_command('RFID_WRITE',
                                   self.cmd_RFID_WRITE,
                                   desc=self.cmd_RFID_WRITE_help)
        self.gcode.register_command('RFID_SCAN',
                                   self.cmd_RFID_SCAN,
                                   desc=self.cmd_RFID_SCAN_help)
        self.gcode.register_command('RFID_STATUS',
                                   self.cmd_RFID_STATUS,
                                   desc=self.cmd_RFID_STATUS_help)
        self.gcode.register_command('RFID_SET_POWER',
                                   self.cmd_RFID_SET_POWER,
                                   desc=self.cmd_RFID_SET_POWER_help)
        self.gcode.register_command('RFID_RESET',
                                   self.cmd_RFID_RESET,
                                   desc=self.cmd_RFID_RESET_help)
        
        # WebSocket API
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint("rfid/status", self._handle_status_request)
        webhooks.register_endpoint("rfid/read", self._handle_read_request)
        webhooks.register_endpoint("rfid/write", self._handle_write_request)
        
    def _build_config(self):
        # Convert scan interval to MCU ticks
        clock = self.mcu.get_query_slot(self.oid)
        rest_ticks = self.mcu.seconds_to_clock(self.scan_interval)
        
        # Build pin list for config command
        # Extract pin strings from pin_params dicts
        pin_strings = []
        for pin_info in self.antenna_pins:
            # pin_info is a dict with 'pin' key containing the pin string (e.g., "PB12")
            pin_strings.append(pin_info['pin'])
        
        # Configure RFID sensor
        # Python sends %s (strings), C receives %u (numbers via enumeration)
        self.mcu.add_config_cmd(
            "config_rfid oid=%d clock=%u spi_oid=%d power=%d "
            "ch1a_pin=%s ch1b_pin=%s ch2a_pin=%s ch2b_pin=%s "
            "ch3a_pin=%s ch3b_pin=%s ch4a_pin=%s ch4b_pin=%s "
            "ch5a_pin=%s ch5b_pin=%s ch6a_pin=%s ch6b_pin=%s "
            "ch7a_pin=%s ch7b_pin=%s ch8a_pin=%s ch8b_pin=%s"
            % tuple([self.oid, rest_ticks, self.spi.get_oid(),
                    self.power_level] + pin_strings))
        
        # Create command wrappers
        cmd_queue = self.mcu.alloc_command_queue()
        self.rfid_start_scan_cmd = self.mcu.lookup_command(
            "rfid_start_scan oid=%c", cq=cmd_queue)
        self.rfid_stop_scan_cmd = self.mcu.lookup_command(
            "rfid_stop_scan oid=%c", cq=cmd_queue)
        self.rfid_read_tag_cmd = self.mcu.lookup_command(
            "rfid_read_tag oid=%c channel=%c", cq=cmd_queue)
        self.rfid_write_tag_cmd = self.mcu.lookup_command(
            "rfid_write_tag oid=%c channel=%c page=%c data=%*s", cq=cmd_queue)
        self.rfid_set_power_cmd = self.mcu.lookup_command(
            "rfid_set_power oid=%c level=%c", cq=cmd_queue)
        self.rfid_get_status_cmd = self.mcu.lookup_command(
            "rfid_get_status oid=%c", cq=cmd_queue)
    
    def _handle_rfid_read_response(self, params):
        channel = params['channel']
        status = params['status']
        
        #self.last_result[channel] = {
        #    'status': status,
        #    'uid': None,
        #    'data': None,
        #    'timestamp': self.reactor.monotonic()
        #}
        logging.info("RFID: read response on channel %d, status: %d",
                    channel, status)
        if self.scanning and channel == 7:
            # Last channel scanned, update scan results
            self._update_scan_results()

    def _handle_rfid_data_response(self, params):
        channel = params['channel']
        uid = params.get('uid', b'')
        data_chunk = params.get('data', b'')
        sequence = params.get('sequence', 0)

        #logging.info("Python: >>> _handle_rfid_data_response called. Channel: %s, Sequence: %s, Chunk Length: %s",
        #             channel, sequence, len(data_chunk))
        #logging.info("Python: Raw data_chunk (first 10 bytes): %s", data_chunk[:10].hex())

        if channel not in self.bulk_tag_data_buffers:
            self.bulk_tag_data_buffers[channel] = {
                'data': b'',
                'last_sequence': -1,
                'uid': b''
            }
        
        current_buffer = self.bulk_tag_data_buffers[channel]
        expected_sequence = current_buffer['last_sequence'] + 1
        
        if sequence == expected_sequence:
            current_buffer['data'] += data_chunk
            current_buffer['last_sequence'] = sequence
            if uid and not current_buffer['uid']:
                current_buffer['uid'] = uid
            logging.info("RFID: Data chunk received for channel %d, sequence %d, current_len %d",
                          channel, sequence, len(current_buffer['data']))
        elif sequence == 0 and current_buffer['last_sequence'] != -1:
            logging.info("RFID: New read cycle started on channel %d (sequence 0). Resetting buffer.", channel)
            current_buffer['data'] = data_chunk
            current_buffer['last_sequence'] = sequence
            current_buffer['uid'] = uid
        else:
            logging.info("RFID: Sequence mismatch on channel %d. Expected %d, got %d. Discarding chunk.",
                            channel, expected_sequence, sequence)
            return
        
        full_tag_data = current_buffer['data'][:144]
        #logging.info(f"Actual data length: {len(full_tag_data)}")
            
        #for i in range(len(full_tag_data)):
        #    logging.info(f"  data[{i}] -> 0x{full_tag_data[i]:02x}")

        hex_str = []
        for i in range(0, len(full_tag_data), 16):
            chunk = full_tag_data[i:i+16]
            hex_chunk = ' '.join(f'{b:02x}' for b in chunk)
            ascii_chunk = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in chunk])
            hex_str.append(f"  0x{i:04x}: {hex_chunk.ljust(47)}  |  {ascii_chunk}  # {i}-{min(i+15, len(full_tag_data)-1)}字节")

        logging.info(
            "RFID: Raw data (hex):\n%s",
            '\n'.join(hex_str)
        )

    
    def _handle_rfid_read_complete(self, params):
        channel = params['channel']
        status = params['status'] # 整体读取状态
        total_bytes = params.get('total_bytes', 0) # C 端报告的应有总字节数
        uid = params.get('uid', b'') # 最终的 UID，以防数据块中没有

        current_buffer = self.bulk_tag_data_buffers.get(channel)

        if current_buffer and status == MI_OK:
            if not current_buffer['uid'] and uid:
                current_buffer['uid'] = uid

            if len(current_buffer['data']) >= total_bytes:
                full_tag_data = current_buffer['data'][:total_bytes]

                tag_info = self._parse_tag_data(current_buffer['uid'], full_tag_data)
                self.tag_data[channel] = tag_info
                self.tag_present[channel] = True
                
                if self.auto_load and tag_info:
                    self._apply_filament_settings(tag_info)
                
                logging.info("RFID: Full Tag data (complete) received on channel %d, status: %d, UID: %s, data_len: %d",
                            channel, status, current_buffer['uid'].hex(), len(full_tag_data))
            else:
                self.tag_data[channel] = None
                self.tag_present[channel] = False
                logging.error("RFID: Incomplete data received for channel %d. Expected %d bytes, got %d.",
                            channel, total_bytes, len(current_buffer['data']))
        else:
            self.tag_data[channel] = None
            self.tag_present[channel] = False
            logging.info("RFID: Tag detected ERROR on channel %d, status: %d",
                        channel, status)
        
        if current_buffer:
            current_buffer['data'] = b''
            current_buffer['last_sequence'] = -1
            current_buffer['uid'] = b''

    
    def _handle_rfid_status_response(self, params):
        self.current_channel = params['channel']
        self.power_level = params['power']
        present_mask = params['present']
        self.scanning = bool(params['scanning'])
        
        # Update tag presence from bitmask
        for i in range(8):
            self.tag_present[i] = bool(present_mask & (1 << i))
    
    def _handle_rfid_write_response(self, params):
        channel = params['channel']
        status = params['status']
        write_count = params['write_count']
        
        if status == MI_OK:
            logging.info("RFID: Write successful on channel %d,  write_count %d", channel, write_count)
        else:
            logging.error("RFID: Write failed on channel %d,  write_count %d", channel, write_count)
    
    def _parse_tag_data(self, uid, data):
        """Parse OpenTag format data from NTAG213"""
        if len(data) < OPENTAG_HEADER_SIZE:
            logging.info("RFID: OPENTAG data too short ")
            return None

        try:
            # Parse header (required fields)
            offset = 0
            version = data[offset]
            offset += 2
            
            #if version != OPENTAG_VERSION:
            #    #logging.warning("RFID: Unsupported OpenTag version %d", version)
            #    logging.info("RFID: Unsupported OpenTag version %d", version)
            #    return None
            
            logging.info(f"parse_tag_data Actual data length: {len(data)}")
            
            for i in range(len(data)):
                logging.info(f"  data[{i}] -> 0x{data[i]:02x}")

            
            # Extract fields
            tag_info = {
                'uid': uid.hex(),
                'version': version,
                'manufacturer': data[offset:offset+16].decode('utf-8').strip('\x00'),
                'material': data[offset+16:offset+32].decode('utf-8').strip('\x00'),
                'color': data[offset+32:offset+64].decode('utf-8').strip('\x00'),
                'diameter': struct.unpack('>H', data[offset+64:offset+66])[0],
                'initial_weight': struct.unpack('>H', data[offset+66:offset+68])[0],
                'nozzle_temp': struct.unpack('>H', data[offset+68:offset+70])[0],
                'bed_temp': struct.unpack('>H', data[offset+70:offset+72])[0],
                'density': struct.unpack('>H', data[offset+72:offset+74])[0],
                'serialNumber': data[offset+74:offset+90].decode('utf-8').strip('\x00'),
            #    'min_nozzle_temp': struct.unpack('<H', data[offset+64:offset+66])[0],
            #    'max_nozzle_temp': struct.unpack('<H', data[offset+66:offset+68])[0],
            #    'print_speed': struct.unpack('<H', data[offset+68:offset+70])[0],
            #    'retract_distance': struct.unpack('<f', data[offset+70:offset+74])[0],
            #    'retract_speed': struct.unpack('<H', data[offset+74:offset+76])[0],
            }

            # Parse optional fields if present
            #if len(data) >= OPENTAG_TOTAL_SIZE:
            #    offset = OPENTAG_HEADER_SIZE
            #    tag_info['serial_number'] = data[offset:offset+16].decode('utf-8').strip('\x00')
            #    tag_info['production_date'] = data[offset+16:offset+24].decode('utf-8').strip('\x00')
            #    tag_info['url'] = data[offset+24:offset+88].decode('utf-8').strip('\x00')
                
            return tag_info
            #return None
            
        except Exception as e:
            logging.error("RFID: Failed to parse tag data: %s", str(e))
            return None
    
    def _apply_filament_settings(self, tag_info):
        """Apply filament settings from tag to printer"""
        try:
            # Set extruder temperature
            if tag_info.get('nozzle_temp'):
                self.gcode.run_script_from_command(
                    f"M104 S{tag_info['nozzle_temp']}")
            
            # Set bed temperature
            if tag_info.get('bed_temp'):
                self.gcode.run_script_from_command(
                    f"M140 S{tag_info['bed_temp']}")
            
            # Set print speed percentage
            #if tag_info.get('print_speed'):
            #    speed_percent = int(tag_info['print_speed'] * 100 / 60)
            #    self.gcode.run_script_from_command(
            #        f"M220 S{speed_percent}")
            
            # Store filament info for reference
            self.gcode.run_script_from_command(
                f"SET_GCODE_VARIABLE MACRO=_FILAMENT_INFO "
                f"VARIABLE=material VALUE='\"{tag_info.get('material', 'Unknown')}\"'")
            self.gcode.run_script_from_command(
                f"SET_GCODE_VARIABLE MACRO=_FILAMENT_INFO "
                f"VARIABLE=color VALUE='\"{tag_info.get('color', 'Unknown')}\"'")
            self.gcode.run_script_from_command(
                f"SET_GCODE_VARIABLE MACRO=_FILAMENT_INFO "
                f"VARIABLE=weight VALUE={tag_info.get('current_weight', 0)}")
            
            #logging.info("RFID: Applied filament settings - Material: %s, Color: %s",
            #            tag_info.get('material'), tag_info.get('color'))
            
            logging.info("RFID: Applied filament settings - Material: %s, Color: %s\n",
                        tag_info.get('material'), tag_info.get('color'))
                        
            logging.info("RFID: Full Tag Data Details:")
            logging.info("  UID: %s", tag_info.get('uid', 'Unknown'))
            logging.info("  Manufacturer: %s", tag_info.get('manufacturer', 'Unknown'))
            logging.info("  Material: %s", tag_info.get('material', 'Unknown'))
            logging.info("  Color: %s", tag_info.get('color', 'Unknown'))
            logging.info("  Diameter (mm): %s", tag_info.get('diameter', 'Unknown'))
            logging.info("  Initial Weight (g): %s", tag_info.get('initial_weight', 'Unknown'))
            logging.info("  Nozzle_temp: %s", tag_info.get('nozzle_temp', 'Unknown'))
            logging.info("  Bed_temp: %s", tag_info.get('bed_temp', 'Unknown'))
            logging.info("  Density: %s", tag_info.get('density', 'Unknown'))
            logging.info("  SerialNumber: %s", tag_info.get('serialNumber', 'Unknown'))
            
        except Exception as e:
            logging.error("RFID: Failed to apply filament settings: %s", str(e))
    
    def start_scanning(self):
        """Start automatic tag scanning"""
        if not self.scanning:
            self.rfid_start_scan_cmd.send([self.oid])
            self.scanning = True
            logging.info("RFID: Started scanning")
    
    def stop_scanning(self):
        """Stop automatic tag scanning"""
        if self.scanning:
            self.rfid_stop_scan_cmd.send([self.oid])
            self.scanning = False
            logging.info("RFID: Stopped scanning")
    
    def read_tag(self, channel):
        """Read tag from specific channel"""
        if channel < 0 or channel >= 8:
            raise self.gcode.error(f"Invalid RFID channel: {channel}")
        
        self.rfid_read_tag_cmd.send([self.oid, channel])
        
        # Wait for response
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        
        return self.tag_data[channel]
    
    def write_tag(self, channel, page, data):
        """Write data to tag"""
        if channel < 0 or channel >= 8:
            raise self.gcode.error(f"Invalid RFID channel: {channel}")
        if page < 4 or page > 39:
            raise self.gcode.error(f"Invalid NTAG213 page: {page}")
        
        self.rfid_write_tag_cmd.send([self.oid, channel, page, data])
    
    def set_power_level(self, level):
        """Set antenna power level (0-3)"""
        if level < 0 or level > 3:
            raise self.gcode.error(f"Invalid power level: {level}")
        
        self.power_level = level
        self.rfid_set_power_cmd.send([self.oid, level])
    
    #def get_status(self):
    #    """Get current RFID status"""
    #    self.rfid_get_status_cmd.send([self.oid])
    #    self.reactor.pause(self.reactor.monotonic() + 0.1)
    #    
    #    return {
    #        'scanning': self.scanning,
    #        'current_channel': self.current_channel,
    #        'power_level': self.power_level,
    #        'tags_present': self.tag_present,
    #        'tag_data': [tag for tag in self.tag_data if tag is not None]
    #    }
    
    # G-code command handlers
    cmd_RFID_READ_help = "Read RFID tag from specified channel"
    def cmd_RFID_READ(self, gcmd):
        channel = gcmd.get_int('CHANNEL', 0, minval=0, maxval=7)
        tag = self.read_tag(channel)
        
        if tag:
            gcmd.respond_info(f"RFID Channel {channel}: {tag.get('material', 'Unknown')} "
                            f"{tag.get('color', 'Unknown')} "
                            f"{tag.get('current_weight', 0):.1f}g")
        else:
            gcmd.respond_info(f"RFID Channel {channel}: No tag detected")
    
    cmd_RFID_WRITE_help = "Write data to RFID tag"
    def cmd_RFID_WRITE(self, gcmd):
        channel = gcmd.get_int('CHANNEL', 0, minval=0, maxval=7)
        page = gcmd.get_int('PAGE', 4, minval=4, maxval=39)
        data = gcmd.get('DATA', '2233')
        #data = b'\xA1\xA2\xA3\xA4' 
        
        if len(data) > 4:
            raise gcmd.error("Data too long for NTAG213 page (max 4 bytes)")
        
        # Pad data to 4 bytes
        data_bytes = data.encode('utf-8')[:4].ljust(4, b'\x00')
        self.write_tag(channel, page, data_bytes)
        gcmd.respond_info(f"RFID: Writing to channel {channel}, page {page}")
    
    cmd_RFID_SCAN_help = "Start or stop automatic RFID scanning"
    def cmd_RFID_SCAN(self, gcmd):
        enable = gcmd.get_int('ENABLE', 1)
        
        if enable:
            self.start_scanning()
            gcmd.respond_info("RFID: Scanning started")
        else:
            self.stop_scanning()
            gcmd.respond_info("RFID: Scanning stopped")
    
    cmd_RFID_STATUS_help = "Get RFID system status"
    def cmd_RFID_STATUS(self, gcmd):
        status = self.get_status()
        
        gcmd.respond_info(f"RFID Status:")
        gcmd.respond_info(f"  Scanning: {status['scanning']}")
        gcmd.respond_info(f"  Power Level: {status['power_level']}")
        gcmd.respond_info(f"  Current Channel: {status['current_channel']}")
        
        for i, present in enumerate(status['tags_present']):
            if present:
                tag = self.tag_data[i]
                if tag:
                    gcmd.respond_info(f"  Channel {i}: {tag.get('material', 'Unknown')} "
                                    f"{tag.get('color', 'Unknown')}")
                else:
                    gcmd.respond_info(f"  Channel {i}: Tag present (unread)")
            else:
                gcmd.respond_info(f"  Channel {i}: Empty")
    
    cmd_RFID_SET_POWER_help = "Set RFID antenna power level (0-3)"
    def cmd_RFID_SET_POWER(self, gcmd):
        level = gcmd.get_int('LEVEL', 2, minval=0, maxval=3)
        self.set_power_level(level)
        
        power_names = ['Low', 'Standard', 'High', 'Maximum']
        gcmd.respond_info(f"RFID: Power level set to {power_names[level]}")
    
    cmd_RFID_RESET_help = "Reset RFID system"
    def cmd_RFID_RESET(self, gcmd):
        self.stop_scanning()
        self.tag_data = [None] * 8
        self.tag_present = [False] * 8
        gcmd.respond_info("RFID: System reset")
    
    # WebSocket API handlers
    def _handle_status_request(self, web_request):
        status = self.get_status()
        web_request.send(status)
    
    def _handle_read_request(self, web_request):
        channel = web_request.get_int('channel', 0)
        tag = self.read_tag(channel)
        web_request.send({'channel': channel, 'tag': tag})
    
    def _handle_write_request(self, web_request):
        channel = web_request.get_int('channel')
        page = web_request.get_int('page')
        data = web_request.get('data', '')
        
        data_bytes = data.encode('utf-8')[:4].ljust(4, b'\x00')
        self.write_tag(channel, page, data_bytes)
        web_request.send({'status': 'ok'})

def load_config(config):
    return RFIDSensor(config)

def load_config_prefix(config):
    return RFIDSensor(config)