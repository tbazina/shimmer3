#/usr/bin/python3

import sys, struct, serial
from pathlib import Path
import argparse
from collections import defaultdict
from datetime import datetime, timedelta
import time
import csv


class ShimmerCaptureEMG():
  def __init__(self, port) -> None:
    # Serial port
    self.port = port

    # // Packet Types// Packet Types
    self.packet_type = {
    'ACK_COMMAND_PROCESSED': 0xFF,
    'GET_SAMPLING_RATE_COMMAND': 0x03,
    'SET_SAMPLING_RATE_COMMAND': 0x05,
    'INQUIRY_COMMAND': 0x01,
    'GET_BUFFER_SIZE_COMMAND': 0x36,
    'BUFFER_SIZE_RESPONSE': 0x35,
    'SET_INFOMEM_COMMAND': 0x8C,
    'GET_INFOMEM_COMMAND': 0x8E,
    'GET_CHARGE_STATUS_LED_COMMAND': 0x32,
    'CHARGE_STATUS_LED_RESPONSE': 0x31,
    'SET_SENSORS_COMMAND': 0x08,
    'SET_EXG_REGS_COMMAND': 0x61,
    'GET_DAUGHTER_CARD_ID_COMMAND': 0x66,
    'DAUGHTER_CARD_ID_RESPONSE': 0x65,
    'GET_EXG_REGS_COMMAND': 0x63,
    'EXG_REGS_RESPONSE': 0x62,
    'GET_SHIMMERNAME_COMMAND': 0x7b,
    'SHIMMERNAME_RESPONSE': 0x7a,
    'START_STREAMING_COMMAND': 0x07,
    'STOP_STREAMING_COMMAND': 0x20,
    'DATA_PACKET': 0x00,
    'SET_RWC_COMMAND': 0x8F,
    'RWC_RESPONSE': 0x90,
    'GET_RWC_COMMAND': 0x91,
    'SET_CONFIGTIME_COMMAND': 0x85,
    'CONFIGTIME_RESPONSE': 0x86,
    'GET_CONFIGTIME_COMMAND': 0x87,
    }

    # EMG gain configuration bytes
    self.emg_gain_config = defaultdict(
      lambda: 0b110, # default 12
      {
        1:  0b001,
        2:  0b010,
        3:  0b011,
        4:  0b100,
        6:  0b000,
        8:  0b101,
        12: 0b110
      }
    )
    # Default gain value
    self.emg_gain_config_default = 12

    # EMG multiplexer settings for both channels
    self.emg_mux_config = defaultdict(
      lambda: 0b1001, # Route INxP and INxN to channel x inputs
      {
        'normal': 0b0000, # Normal electrode input (default)
        'shorted': 0b0001, # Input shorted (for offset measurements and power down channel)
        'test': 0b0101, # Test signal
        'measure_EMG': 0b1001, # Route INxP and INxN to channel x inputs
      }
    )

    # EMG data rate configuration bytes
    self.emg_data_rate_config = defaultdict(
      lambda: 0x02, # 500 Hz
      {
        125: 0x00,
        250: 0x01,
        500: 0x02,
        1000: 0x03,
        2000: 0x04,
        4000: 0x05,
      }
    )

    # EMG Right-Leg Drive (Common-mode Rejection) settings
    # RLD2N, RLD2P, RLD1N and RLD1P should be 0 for both chips
    self.emg_rld_config = 0b0000

    # PDB_RLD: RLD buffer power bit determines the RLD buffer power state
    # 0 = powered down (default), 1 = enabled
    # RLD_LOFF_SENSE: RLD lead-off sense function
    # 0 = RLD lead-off sense is disabled (default)
    # 1 = RLD lead-off sense is enabled
    self.emg_pbd_rld_loff_sense_config = {
      'on': 0b10,
      'off': 0b00
    }
    
    # RLDREF_INT: RLDREF signal
    # 0 = RLDREF signal fed externally
    # 1 = RLDREF signal (AVDD – AVSS) / 2 generated internally (default)
    self.emg_rldref_int_config = {
      'external': 0,
      'internal': 1
    }

    # INT_TEST: Test signal selection
    # This bit determines whether the test signal is turned on or off.
    # 0 = Off (default)
    # 1 = On; amplitude = ±(VREFP – VREFN) / 2400
    # TEST_FREQ: Test signal frequency
    # This bit determines the test signal frequency.
    # 0 = At dc (default)
    # 1 = Square wave at 1 Hz
    self.test_signal_config = {
      'DC': 0b10,
      'square': 0b11,
      'off': 0b00
    }

    # EMG 24 bit sensor activation
    self.emg_24bit = [0x18, 0x00, 0x00]

    # Signal calibration parameters
    self.adc_offset_ch1 = 0
    self.adc_offset_ch2 = 0
    self.adc_sensitivity = 2420 / (2**23 - 1)
  
    # Dictionary to store possible values for charge status LED 
    self.charge_status_dict = defaultdict(
      lambda: 'UNDEFINED', {
        0: 'FULL',
        1: 'MEDIUM',
        2: 'LOW',
      }
      )

    # Signal calibrated indicator
    self.signal_calibrated = False
  
  def open_port(self):
    self.ser = serial.Serial(
      port=self.port,
      baudrate=115200,
      timeout=10.0,
      write_timeout=10.0,
      )
    print('Port {} opened: {}'.format(self.port, self.ser.is_open))
    self.ser.reset_input_buffer()
    return self.ser

  def __enter__(self):
    return self.open_port()

  def close_port(self):
    if self.ser.is_open:
      self.ser.close()
      print('Port {} opened: {}'.format(self.port, self.ser.is_open))

  def __exit__(self, exception_type, exception_value, exception_traceback):
    # Exception handling
    self.close_port()
    # if isinstance(exception_value, serial.SerialTimeoutException):
    #   print('Write timeout exceeded!')
    #   print("nTraceback: {}".format(exception_traceback))
      
  def wait_for_ack(self):
    """
    Acknowledge the error-free receipt of transmitted data.
    """
    if self.ser.is_open:
      ddata = ""
      ack = struct.pack(
        'B', self.packet_type['ACK_COMMAND_PROCESSED']
        )
      while ddata != ack:
          ddata = self.ser.read(1)
    else:
      sys.exit('Serial port not open!')
  
  def set_sampling_rate(self, sampling_rate):
    """Set Shimmer3 sampling rate to desired value.

    Args:
        sampling_rate (int): Sampling rate in Hz.
    """
    try:
      sampling_rate = int(sampling_rate)
      if not(0.6<=sampling_rate<=1024):
        raise ValueError
      if self.ser.is_open:
        # Calculate from Hz to ms
        sampling_period = round(32768/sampling_rate)
        # Send sampling rate command bitwise & and >> due to alignment issues
        self.ser.write(struct.pack(
          'BBB',
          self.packet_type['SET_SAMPLING_RATE_COMMAND'],
          (sampling_period&0xFF),
          ((sampling_period&0xFF00)>>8)
          ))
        self.wait_for_ack()
        # Read and print set sampling rate
        self.get_sampling_rate(echo=True)
      else:
        sys.exit('Serial port not open!')
    except ValueError as e:
      sys.exit(
        'set_sampling_rate not an integer or between 1 and 1024 Hz'
        '{}'.format(e)
        )

  def get_sampling_rate(self, echo=False):
    """Read and print sampling rate"""
    if self.ser.is_open:
      # Send sampling rate command
      self.ser.write(struct.pack(
        'B', self.packet_type['GET_SAMPLING_RATE_COMMAND']
        ))
      self.wait_for_ack()
      # Read incoming data 1 identifying byte, 2 for uint sampling rate
      data = self.ser.read(size=3)
      # print(data[0])
      clock_wait = struct.unpack('H', data[1:3])[0]
      self.sampling_rate = 32768/clock_wait
      if echo:
        print('Shimmer sampling rate: {:.4f} Hz'.format(
          self.sampling_rate
          ))
      return self.sampling_rate
    else:
      sys.exit('Serial port not open!')

  def get_buffer_size(self, echo=False):
    """Read and print buffer size"""
    if self.ser.is_open:
      # Send get buffer size command
      self.ser.write(struct.pack(
        'B', self.packet_type['GET_BUFFER_SIZE_COMMAND']
        ))
      self.wait_for_ack()
      # Read incoming data 1 identifying byte, 2 for uint sampling rate
      data = self.ser.read(size=2)
      if data[0] == self.packet_type['BUFFER_SIZE_RESPONSE']:
        self.buffer_size = data[1]
        if echo:
          print('Buffer size: {}'.format(self.buffer_size))
        return self.buffer_size
      else:
        print('Did not recieve BUFFER_SIZE_RESPONSE')
    else:
      sys.exit('Serial port not open!')
  
  def get_charge_status_led(self, echo=False):
    """Read and print charge status LED"""
    # TODO: TEST for YELLOW and RED
    if self.ser.is_open:
      # send get charge status led command
      self.ser.write(struct.pack(
        'B', self.packet_type['GET_CHARGE_STATUS_LED_COMMAND']
        ))
      self.wait_for_ack()
      # Read incoming data 1 identifying byte, 1 charge status
      data = self.ser.read(size=2)
      if data[0] == self.packet_type['CHARGE_STATUS_LED_RESPONSE']:
        charge_status_num = data[1]
        self.charge_status = self.charge_status_dict[charge_status_num]
        if echo:
          print('Charge status: {}'.format(self.charge_status))
        return self.charge_status
      else:
        print('Did not recieve CHARGE_STATUS_LED_RESPONSE')
    else:
      sys.exit('Serial port not open!')

  def get_id_and_rev(self, echo=True):
    """Get the daughter card ID byte (SR number) and Revision number"""
    #TODO: Add the GET_SHIMMERNAME functionality
    if self.ser.is_open:
      self.ser.write(struct.pack(
        'BBB', self.packet_type['GET_DAUGHTER_CARD_ID_COMMAND'], 0x02, 0x00
        ))
      self.wait_for_ack()
      # Read incoming data byte 3 - serial number, byte 4 revision
      data = self.ser.read(size=4)
      if data[0] == self.packet_type['DAUGHTER_CARD_ID_RESPONSE']:
        self.serial_number = data[2]
        self.revision_number = data[3]
        if echo:
          print('Device: SR{}-{}'.format(self.serial_number, self.revision_number))
        return (self.serial_number, self.revision_number)
      else:
        print('Did not recieve DAUGHTER_CARD_ID_RESPONSE')
    else:
      sys.exit('Serial port not open!')

  def set_emg_gain(self, gain):
    """Set gain to variable"""
    # Check if valid gain input
    if gain in self.emg_gain_config.keys():
      self.emg_gain = gain
    else:
      self.emg_gain = self.emg_gain_config_default
    print('EMG gain: {}'.format(self.emg_gain))
    self.emg_gain_packet = self.emg_gain_config[self.emg_gain]
    # print('{:03b}'.format(self.emg_gain_packet))
    return self.emg_gain

  def set_emg_data_rate(self, emg_data_rate):
    """Set emg chip data rate (samples per second) to variable"""
    # Check if valid data rate input
    if emg_data_rate in self.emg_data_rate_config.keys():
      self.emg_data_rate = emg_data_rate
    else:
      self.sampling_rate = self.get_sampling_rate()
      # Set the data rate to first value higher than Shimmer sampling rate
      self.emg_data_rate = min(
        [i for i in self.emg_data_rate_config.keys() if i > self.sampling_rate]
        )
    print('EMG chip data rate: {} Hz'.format(self.emg_data_rate))
    self.emg_data_rate_packet = self.emg_data_rate_config[self.emg_data_rate]
    return self.emg_data_rate
  
  def get_emg_registers(self, chip_num=0, echo=False):
    #TODO: set first bit on CH1SET and CH2SET on chip 2 to 1 (disable chip 2)
    """Get byte values for all 10 registers for chip_num on EMG unit"""
    if chip_num not in [0, 1]:
      sys.exit('Wrong chip number specified. Must be 0 or 1')
    if self.ser.is_open:
      self.ser.write(struct.pack(
        'B'*4, self.packet_type['GET_EXG_REGS_COMMAND'], chip_num, 0, 10
        ))
      self.wait_for_ack()
      # Read incoming data bytes (EXG_REGS_RESPONSE + number of bytes + 10 registers)
      data = self.ser.read(size=12)
      if data[0] == self.packet_type['EXG_REGS_RESPONSE']:
        emg_regs = list(struct.unpack('B'*10, data[2:]))
        # Store only chip1 registers
        if chip_num == 0:
          self.emg_regs = emg_regs
        if echo:
          print('EMG register settings for chip {}:\n'.format(chip_num+1),
                '\tCONFIG1: {:08b}\n'.format(emg_regs[0]),
                '\tCONFIG2: {:08b}\n'.format(emg_regs[1]),
                '\tLOFF: {:08b}\n'.format(emg_regs[2]),
                '\tCH1SET: {:08b}\n'.format(emg_regs[3]),
                '\tCH2SET: {:08b}\n'.format(emg_regs[4]),
                '\tRLD_SENS: {:08b}\n'.format(emg_regs[5]),
                '\tLOFF_SENS: {:08b}\n'.format(emg_regs[6]),
                '\tLOFF_STAT: {:08b}\n'.format(emg_regs[7]),
                '\tRESP1: {:08b}\n'.format(emg_regs[8]),
                '\tRESP2: {:08b}\n'.format(emg_regs[9]),
          )
        return emg_regs
      else:
        print('Did not recieve EXG_REGS_RESPONSE')
    else:
      sys.exit('Serial port not open!')

  def power_down_chip_2(self):
    """
    Chip 2 should be powered down for EMG signal acquisition.
    Input multiplexer should be set to input shorted configuration.
    Bit 7 in CH1SET and CH2SET bytes should be set to 1 (Channel x power-down).
    """
    # Get EMG registers for chip 2
    chip_num = 1
    # self.get_emg_registers(chip_num=chip_num, echo=True)
    self.chip2_emg_regs = [None]*10
    # CONFIG1 byte - Data Rate
    self.chip2_emg_regs[0] = self.emg_data_rate_packet
    # CONFIG2 byte - Test signals
    self.chip2_emg_regs[1] = (
      0b10101000 | self.test_signal_config['off']
    )
    # LOFF byte
    self.chip2_emg_regs[2] = 0b00010000
    # CH1SET - Bit7 to 1, gain to default and mux to shorted
    self.chip2_emg_regs[3] = (
      0b1 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config['shorted']
      )
    # CH2SET - Bit7 to 1, gain to default and mux to shorted
    self.chip2_emg_regs[4] = (
      0b1 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config['shorted']
      )
    # RLD_SENS byte - all 8 bits to 0
    self.chip2_emg_regs[5] = (
      0b00 << 6 | self.emg_pbd_rld_loff_sense_config['off'] << 4 | 
      self.emg_rld_config
    )
    # LOFF_SENS and LOFF_STAT bytes
    self.chip2_emg_regs[6], self.chip2_emg_regs[7] = 0b0, 0b0
    # RESP1 byte
    self.chip2_emg_regs[8] = (
      0b10
    )
    # RESP 2 byte
    self.chip2_emg_regs[9] = (
      self.emg_rldref_int_config['external'] << 1 | 0b1
    )
    # print('EMG register settings for chip {}:\n'.format(chip_num+1),
    #       '\tCONFIG1: {:08b}\n'.format(self.chip2_emg_regs[0]),
    #       '\tCONFIG2: {:08b}\n'.format(self.chip2_emg_regs[1]),
    #       '\tLOFF: {:08b}\n'.format(self.chip2_emg_regs[2]),
    #       '\tCH1SET: {:08b}\n'.format(self.chip2_emg_regs[3]),
    #       '\tCH2SET: {:08b}\n'.format(self.chip2_emg_regs[4]),
    #       '\tRLD_SENS: {:08b}\n'.format(self.chip2_emg_regs[5]),
    #       '\tLOFF_SENS: {:08b}\n'.format(self.chip2_emg_regs[6]),
    #       '\tLOFF_STAT: {:08b}\n'.format(self.chip2_emg_regs[7]),
    #       '\tRESP1: {:08b}\n'.format(self.chip2_emg_regs[8]),
    #       '\tRESP2: {:08b}\n'.format(self.chip2_emg_regs[9]),
    # )
    # Write configuration to chip 2 register
    if self.ser.is_open:
      # Send set registers command, chip to write to, starting byte,
      # number of bytes to write and unpack a list with registers to write
      self.ser.write(struct.pack(
        'B'*14, self.packet_type['SET_EXG_REGS_COMMAND'], chip_num, 0, 10,
        *self.chip2_emg_regs
        ))
      self.wait_for_ack()
      print('Chip {} powered down for EMG measurement!'.format(chip_num+1))
      return self.chip2_emg_regs
    else:
      sys.exit('Serial port not open!')

  def activate_emg_sensors(self):
    """Set the 24 bit EXG sensors"""
    sensors = [self.packet_type['SET_SENSORS_COMMAND']] + self.emg_24bit
    if self.ser.is_open:
      # Send set sensors command and emg_24 bit bytes to activate sensor
      self.ser.write(struct.pack('B'*4, *sensors))
      self.wait_for_ack()
      print('24 bit EMG sensor activated!')
    else:
      sys.exit('Serial port not open!')

  def set_emg_registers(self, test_signal=False):
    """Set the EMG registers for chip 1 (EMG signal or test signal)"""
    # Get EMG registers for chip 0
    chip_num = 0
    if test_signal:
      # Set mux config and test signal byte
      mux = 'test'
      test = 'square'
    else:
      mux = 'measure_EMG'
      test = 'off'
    # Initialize list for EMG registers
    self.chip1_emg_regs = [None]*10
    # CONFIG1 byte - Data Rate
    self.chip1_emg_regs[0] = self.emg_data_rate_packet
    # CONFIG2 byte - Test signals
    self.chip1_emg_regs[1] = (
      0b10101000 | self.test_signal_config[test]
    )
    # LOFF byte
    self.chip1_emg_regs[2] = 0b00010000
    # CH1SET - Bit7 to 0, gain to defined and mux to test/measure_EMG
    self.chip1_emg_regs[3] = (
      0b0 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config[mux]
      )
    # CH2SET - Bit7 to 0, gain to defined and mux to test/measure_EMG
    self.chip1_emg_regs[4] = (
      0b0 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config[mux]
      )
    # RLD_SENS byte - PBD_RLD to on
    self.chip1_emg_regs[5] = (
      0b00 << 6 | self.emg_pbd_rld_loff_sense_config['on'] << 4 | 
      self.emg_rld_config
    )
    # LOFF_SENS and LOFF_STAT bytes
    self.chip1_emg_regs[6], self.chip1_emg_regs[7] = 0b0, 0b0
    # RESP1 byte
    self.chip1_emg_regs[8] = (
      0b10
    )
    # RESP 2 byte - RLDREF_INT to internal
    self.chip1_emg_regs[9] = (
      self.emg_rldref_int_config['internal'] << 1 | 0b1
    )
    # print('EMG register settings for chip {}:\n'.format(chip_num+1),
    #       '\tCONFIG1: {:08b}\n'.format(self.chip1_emg_regs[0]),
    #       '\tCONFIG2: {:08b}\n'.format(self.chip1_emg_regs[1]),
    #       '\tLOFF: {:08b}\n'.format(self.chip1_emg_regs[2]),
    #       '\tCH1SET: {:08b}\n'.format(self.chip1_emg_regs[3]),
    #       '\tCH2SET: {:08b}\n'.format(self.chip1_emg_regs[4]),
    #       '\tRLD_SENS: {:08b}\n'.format(self.chip1_emg_regs[5]),
    #       '\tLOFF_SENS: {:08b}\n'.format(self.chip1_emg_regs[6]),
    #       '\tLOFF_STAT: {:08b}\n'.format(self.chip1_emg_regs[7]),
    #       '\tRESP1: {:08b}\n'.format(self.chip1_emg_regs[8]),
    #       '\tRESP2: {:08b}\n'.format(self.chip1_emg_regs[9]),
    # )
    # Write configuration to chip 1 register
    if self.ser.is_open:
      # Send set registers command, chip to write to, starting byte,
      # number of bytes to write and unpack a list with registers to write
      self.ser.write(struct.pack(
        'B'*14, self.packet_type['SET_EXG_REGS_COMMAND'], chip_num, 0, 10,
        *self.chip1_emg_regs
        ))
      self.wait_for_ack()
      print('Chip {} EMG register settings written!'.format(chip_num+1))
      return self.chip1_emg_regs
    else:
      sys.exit('Serial port not open!')

  def start_streaming_EMG(
    self, test_signal=False, signal_length=200,
    echo=False,
    calibrate_test_signal=5,
    save_to_file = 'data',
    ):
    """
    Power off chip 2,
    Set chip 1 settings,
    Start streaming EMG signal
    """
    # Calculate calibration constant (ADC_offset = 0)
    calibration_constant = self.adc_sensitivity / self.emg_gain
    print('Calibration constant: {}'.format(calibration_constant))
    # Calibrate data stream first using test signal
    if calibrate_test_signal and not self.signal_calibrated:
      self.echo = echo
      echo = False
      self.signal_length = signal_length
      signal_length = round(self.sampling_rate * calibrate_test_signal)
      self.test_signal = test_signal
      test_signal = True
      self.save_to_file = save_to_file
      save_to_file = False
      print(
        f'Performing signal calibration ... please wait '
        f'{calibrate_test_signal} s!'
        )
    # Activate EMG sensors
    self.activate_emg_sensors()
    # Power down chip 2 and print register settings
    self.power_down_chip_2()
    self.get_emg_registers(chip_num=1, echo=True)
    # Set the chip 1 configuration registers
    self.set_emg_registers(test_signal)
    self.get_emg_registers(chip_num=0, echo=True)
    # Send start streaming command
    # try:
    self.send_streaming_command('start', echo=True)
    # Read incoming data
    # 1 byte packet type, 3 bytes timestamp, 14 bytes EMG data
    framesize = 18
    # Firmware clock in 1/32768 sec
    clock_step = 32768
    self.c1ch1, self.c1ch2 = list(), list()
    # Print header
    header = ['ID', 'System time', 'Shimmer time', 'Ch1 [mV]', 'Ch2 [mV]']
    save_data = [header]
    if echo: print(
      f'ID\tSystem time\t\t\tShimmer time\t\tCh1 [mV]\tCh2 [mV]'
      )
    for sig_iter in range(signal_length):
      data = self.ser.read(size=framesize)
      system_dt = datetime.now()
      if data[0] == self.packet_type['DATA_PACKET']:
        # Convert bytes to internal clock_ticks
        clock_ticks = int.from_bytes(data[1:4], 'little')
        # Convert to timestamp in secs and datetime
        timestamp = clock_ticks / clock_step
        dt = datetime.fromtimestamp(timestamp).time()
        # Convert chip 1, channel 1 and 2 data to integer values 
        c1ch1 = int.from_bytes(data[5:8], byteorder='big', signed=True)
        c1ch2 = int.from_bytes(data[8:11], byteorder='big', signed=True)
        # Calibrate emg channels using constant and offset:
        c1ch1 = c1ch1 * calibration_constant - self.adc_offset_ch1
        c1ch2 = c1ch2 * calibration_constant - self.adc_offset_ch2
        # Append results to list
        self.c1ch1.append(c1ch1)
        self.c1ch2.append(c1ch2)
        save_data.append([sig_iter, str(system_dt), str(dt), c1ch1, c1ch2])
        if echo:
          print(f'{sig_iter:<6d}\t{str(system_dt):26}\t{str(dt):15}\t{c1ch1:+14.4f}\t{c1ch2:+14.4f}')
      else:
        print(data)
        sys.exit('Did not recieve DATA_PACKET')
    # Send stop streaming command
    self.send_streaming_command('stop', echo=True)
    # Necessary to wait a bit before flushing input 
    time.sleep(0.1)
    self.ser.reset_input_buffer()
    if calibrate_test_signal and not self.signal_calibrated:
      self.calibrate_test_signal()
    if save_to_file:
      with open(f'{save_to_file}.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f, delimiter=';', quoting=csv.QUOTE_NONNUMERIC)
        # Write captured data with header to file
        writer.writerows(save_data)
        print(f'Captured EMG data saved to {save_to_file}.csv!')
    return (self.c1ch1, self.c1ch2) 
    # except KeyboardInterrupt:
  
  def send_streaming_command(self, command='start', echo=False):
    """Send start streaming command to Shimmer3"""
    if command == 'start':
      command_type = 'START_STREAMING_COMMAND'  
      print_msg = 'Starting data stream!'
    else:
      command_type = 'STOP_STREAMING_COMMAND'
      print_msg = 'Stopping data stream!'
    if self.ser.is_open:
      self.ser.write(struct.pack(
        'B', self.packet_type[command_type]
        ))
      self.wait_for_ack()
      if echo: print(print_msg)
    else:
      sys.exit('Serial port not open!')

  def calibrate_test_signal(self):
    """Calibrate ADC_offset using square test signal"""
    # Retain only values with amplitude near -1 and 1
    self.c1ch1 = [i for i in self.c1ch1 if abs(abs(i) - 1) < 0.1]
    self.c1ch2 = [i for i in self.c1ch2 if abs(abs(i) - 1) < 0.1]
    # Keep same number of positive and negative values in list
    pos_count = len([i for i in self.c1ch1 if i >= 0])
    while pos_count != len(self.c1ch1) - pos_count:
      self.c1ch1.pop()
      pos_count = len([i for i in self.c1ch1 if i >= 0])
    pos_count = len([i for i in self.c1ch2 if i >= 0])
    while pos_count != len(self.c1ch2) - pos_count:
      self.c1ch2.pop()
      pos_count = len([i for i in self.c1ch2 if i >= 0])
    # Calculate ADC offset as signal mean value
    self.adc_offset_ch1 = sum(self.c1ch1) / len(self.c1ch1)
    self.adc_offset_ch2 = sum(self.c1ch2) / len(self.c1ch2)
    self.signal_calibrated = True
    print('Calibration done!')
    print('ADC offset for Channel 1: {}'.format(self.adc_offset_ch1))
    print('ADC offset for Channel 2: {}'.format(self.adc_offset_ch2))
    self.start_streaming_EMG(
      test_signal=self.test_signal,
      signal_length=self.signal_length,
      echo=self.echo,
      save_to_file=self.save_to_file
      )
    return

  def synchronise_system_time(self, ntimes=5, echo=False):
    """Get real world clock from shimmer"""
    # Firmware clock in 1/32768 sec
    if echo: print('Synchronising Shimmer time with system time!')
    clock_step = 32768
    if self.ser.is_open:
      for i in range(ntimes):
        self.ser.write(struct.pack(
          'B', self.packet_type['GET_RWC_COMMAND']
          ))
        self.wait_for_ack()
        # Read 9 bytes - first response and 8 system time in clock time
        data = self.ser.read(9)
        if data[0] == self.packet_type['RWC_RESPONSE']:
          # Capture system time
          system_dt = datetime.now()
          # Convert bytes to internal clock_ticks
          clock_ticks = int.from_bytes(data[1:], 'little')
          # Convert to timestamp in secs
          timestamp = clock_ticks / clock_step
          # print(clock_time.to_bytes(8, 'little'))
          dt = datetime.fromtimestamp(timestamp)
          system_timestamp = system_dt.timestamp()
          print(f"Shimmer3 time: {dt},\t timestamp: {timestamp},\t ")
          print(f"System time:   {system_dt},\t timestamp: {system_timestamp}\t")
          # Sending set real world clock command
          self.ser.write(struct.pack(
            'B'*9,
            self.packet_type['SET_RWC_COMMAND'],
            *((round(datetime.now().timestamp()*clock_step)).to_bytes(8, 'little'))
            ))
          self.wait_for_ack()
          # Loop every second
          time.sleep(1 - (datetime.now().timestamp() % 1))
        else:
          print('Did not recieve RWC_RESPONSE!')
    else:
      sys.exit('Serial port not open!')
    


if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description=
   'Configure and capture EMG signal from Shimmer3 device. '
   '\nFirst specify the serial port of the device you wish to connect to! '
   '\nExample usage:'
   '\n\tshimmer_capture_EMG.py /dev/rfcomm0',
   formatter_class=argparse.RawDescriptionHelpFormatter
  )
  parser.add_argument('serial_port', help='serial port of the device')
  # Shimmer sampling rate
  parser.add_argument(
    '-ssr', '--set_sampling_rate',
    type=int,
    help='an integer to set the Shimmer3 sampling rate (Hz)'
    )
  # parser.add_argument(
  #   '-gsr', '--get_sampling_rate',
  #   action='store_true',
  #   help='get the Shimmer3 sampling rate'
  #   )
  # parser.set_defaults(get_sampling_rate=False)
  # EMG chip data rate
  parser.add_argument(
    '-ser', '--set_emg_data_rate',
    type=int,
    help=
    'an integer to set the EMG chip sampling rate (Hz)'
    '[125, 250, 500, 1000, 2000, 4000]'
    )
  # Ddefault based on  Shimmer sampling rate
  parser.set_defaults(set_emg_data_rate=1)
  # Real world clock
  # parser.add_argument(
  #   '-sst', '--synchronise_system_time',
  #   action='store_true',
  #   help='synchronise Shimmer3 internal clock with systme time'
  #   )
  # parser.set_defaults(synchronise_system_time=False)
  # Buffer
  # parser.add_argument(
  #   '-gbs', '--get_buffer_size',
  #   action='store_true',
  #   help='get the Shimmer3 buffer size'
  #   )
  # parser.set_defaults(get_buffer_size=False)
  # Gain
  parser.add_argument(
    '-seg', '--set_emg_gain',
    type=int,
    help='an integer to set the Shimmer3 EMG gain [1, 2, 3, 4, 6, 8, 12], default: 12'
    )
  parser.set_defaults(set_emg_gain=12)
  # Charge status
  # parser.add_argument(
  #   '-gcs', '--get_charge_status',
  #   action='store_true',
  #   help='get the Shimmer3 LED charge status'
  #   )
  # parser.set_defaults(get_charge_status=False)
  # Serial and Revision number
  # parser.add_argument(
  #   '-gsn', '--get_serial_number',
  #   action='store_true',
  #   help='get the Shimmer3 serial and revision number'
  #   )
  # parser.set_defaults(get_serial_number=False)
  # EMG Registers for chip 1
  parser.add_argument(
    '-ger', '--get_emg_registers',
    action='store_true',
    help='get all 10 (0-9) registers for ADS1292R chip 1 on EMG unit'
    )
  parser.set_defaults(get_emg_registers=False)
  # Stream test signal - square wave at 1 Hz
  parser.add_argument(
    '-sts', '--stream_test_signal',
    action='store_true',
    help='stream square 1Hz test signal instead of EMG sensor data'
    )
  parser.set_defaults(stream_test_signal=False)
  # Start streaming to BT
  parser.add_argument(
    '-sse', '--start_streaming_emg',
    type=int,
    help='configure Shimmer3 and sample -sse EMG sensor data'
    )
  # Calibrate using square test signal before streaming
  parser.add_argument(
    '-cts', '--calibrate_test_signal',
    type=int,
    help='Calibrate EMG sensor data before streaming with -cts seconds'
    )
  # Save data to file with name
  parser.add_argument(
    '-stf', '--save_to_file',
    help='name of file to save captured data to (without extension)')
  args = parser.parse_args()

  Shimmer = ShimmerCaptureEMG(args.serial_port)
  try: 
    with Shimmer as sh:
      Shimmer.get_id_and_rev(echo=True)
      Shimmer.get_charge_status_led(echo=True)
      Shimmer.get_sampling_rate(echo=True)
      Shimmer.get_buffer_size(echo=True)
      Shimmer.synchronise_system_time(echo=True)
      if args.set_sampling_rate:
        Shimmer.set_sampling_rate(sampling_rate=args.set_sampling_rate)
      if args.set_emg_data_rate:
        Shimmer.set_emg_data_rate(args.set_emg_data_rate)
      if args.set_emg_gain:
        Shimmer.set_emg_gain(args.set_emg_gain)
      if args.get_emg_registers:
        Shimmer.get_emg_registers(echo=True)
      if args.start_streaming_emg:
        Shimmer.start_streaming_EMG(
          signal_length=args.start_streaming_emg,
          test_signal=args.stream_test_signal,
          echo=True,
          calibrate_test_signal=args.calibrate_test_signal,
          save_to_file = args.save_to_file
          )
  except serial.SerialTimeoutException:
    sys.exit('Write timeout exceeded!')
  except serial.SerialException:
    sys.exit('The device not found (check serial_port)!')