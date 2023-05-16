#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import minimalmodbus
import serial
from std_msgs.msg import Float32, String

# General constants
BITS_PER_DATA = 16
MM_PER_UNIT = 0.1
# MODBUS constants
MODBUS_SLAVE_ID = 26
# RS-485 constants
RS485_NODE_ID = 1
RS485_HEADER = bytearray(b'\xaa\x53\x0b\x01')
RS485_FOOTER = bytearray(b'\xae')
RS485_BYTES_SIZE = 14
RS485_HEADER.append(RS485_NODE_ID)
# RS-232 constants
RS232_HEADER = bytearray(b'\xaa\x53\x29\x01\x00')
RS232_FOOTER = bytearray(b'\xae')
RS232_BYTES_SIZE = 44

class TzbotMagDriver(Node):
	def __init__(self):
		super().__init__('tzbot_mag_driver')
		self.get_logger().info("tzbot_mag_driver node has started.")
		self.setupParam()
		self.setupSerial()
		self.right_deviation_pub = self.create_publisher(Float32, 'deviation_right', 10)
		self.middle_deviation_pub = self.create_publisher(Float32, 'deviation_middle', 10)
		self.left_deviation_pub = self.create_publisher(Float32, 'deviation_left', 10)
		self.cell_state_pub = self.create_publisher(String, 'cell_state', 10)
		self.create_timer(1/self._frequency, self.loop, callback_group=ReentrantCallbackGroup())


	def setupParam(self):
		self.declare_parameter('communication_interface', 'modbus')
		self.declare_parameter('frequency', 50)
		self.declare_parameter('serial_port', '/dev/ttyS0')
		self.declare_parameter('baudrate', 115200)
		self._comm_interface = self.get_parameter('communication_interface').value
		self._serial_port = self.get_parameter('serial_port').value
		self._baudrate = self.get_parameter('baudrate').value
		self._frequency = self.get_parameter('frequency').value		
		if self._comm_interface not in ['modbus', 'rs485','rs232']:
			self.get_logger().error('Unsupported communication interface : ' + self._comm_interface)
			rclpy.shutdown()
		if self._baudrate not in [9600, 19200, 38400, 115200]:
			self.get_logger().error('Unsupported baudrate : ' + str(self._baudrate))
			rclpy.shutdown()


	def setupSerial(self):
		self._ser = None
		if self._comm_interface == 'modbus':
			try:
				self._ser = minimalmodbus.Instrument(self._serial_port, MODBUS_SLAVE_ID) # Serial Port to read the data from & slave address (in decimal)
				self._ser.serial.baudrate = self._baudrate  # Rate at which the information is shared to the communication channel		
				self._ser.serial.bytesize = 8  # Number of bits per byte
				self._ser.serial.parity   = serial.PARITY_NONE # No parity checking
				self._ser.serial.stopbits = 1 # Number of stop bits
				self._ser.serial.timeout  = 1.0 # Read timeout value in seconds
				self._ser.clear_buffers_before_each_transaction = True
				self.get_logger().info('Connected to serial port ' + self._serial_port + ' at ' + str(self._baudrate) + ' baudrate')
			except ValueError as e:
				self.get_logger().error('Error opening serial: %r' % e)
				rclpy.shutdown()
		elif self._comm_interface in ['rs485','rs232']:
			try:
				self._ser = serial.Serial()
				self._ser.port = self._serial_port # Serial Port to read the data from
				self._ser.baudrate = self._baudrate # Rate at which the information is shared to the communication channel		
				self._ser.parity = serial.PARITY_NONE # No parity checking
				self._ser.stopbits = serial.STOPBITS_ONE # Number of stop bits
				self._ser.bytesize = serial.EIGHTBITS # Number of bits per byte
				self._ser.timeout = 1.0 # Read timeout value in seconds
				self._ser.xonxoff = False # Disable software flow control
				self._ser.rtscts = False # Disable hardware (RTS/CTS) flow control
				self._ser.dsrdtr = False # Disable hardware (DSR/DTR) flow control
			except ValueError as e:
				self.get_logger().error('Error setting serial port: %r' % e)
				rclpy.shutdown()

	def loop(self):
		if self._comm_interface == 'modbus':
			try:
				data = self._ser.read_registers(1000, 4, functioncode=4)
				# r = data[0].hex()
				rgt_dev = self.twos_complement(hex(data[0]),BITS_PER_DATA) * MM_PER_UNIT
				mid_dev = self.twos_complement(hex(data[1]),BITS_PER_DATA) * MM_PER_UNIT		
				lft_dev = self.twos_complement(hex(data[2]),BITS_PER_DATA) * MM_PER_UNIT
				cell_state = str(format(int(hex(data[3]), base=BITS_PER_DATA), '015b'))
				self.right_deviation_pub.publish(Float32(data=rgt_dev))
				self.middle_deviation_pub.publish(Float32(data=mid_dev))
				self.left_deviation_pub.publish(Float32(data=lft_dev))
				self.cell_state_pub.publish(String(data=cell_state))
			except Exception as e:
				self.get_logger().error('Error: %r' % e)
		elif self._comm_interface in ['rs485','rs232']:
			# if serial connection is closed, then open it
			if not self._ser.is_open:
				try:
					self._ser.open()
					self.get_logger().info('Connected to serial port ' + self._serial_port + ' at ' + str(self._baudrate) + ' baudrate')
				except serial.SerialException as e:
					self.get_logger().error('Error opening serial: %r' % e)
					rclpy.shutdown()
			# if serial connection is opened, start reading from serial port
			else:
				self._ser.reset_input_buffer()
				if self._comm_interface == 'rs485':
					# self._ser.flushInput()
					data = self._ser.read_until(bytes(RS485_FOOTER))
					# self.get_logger().info(data.hex(':'))
					if len(data)==RS485_BYTES_SIZE and data[0:5]==RS485_HEADER and data[-1:]==RS485_FOOTER:
						rgt_dev = self.twos_complement(data[5:7].hex(),BITS_PER_DATA) * MM_PER_UNIT
						mid_dev = self.twos_complement(data[7:9].hex(),BITS_PER_DATA) * MM_PER_UNIT		
						lft_dev = self.twos_complement(data[9:11].hex(),BITS_PER_DATA) * MM_PER_UNIT
						cell_state = str(format(int(data[11:13].hex(), base=BITS_PER_DATA), '015b'))
						self.right_deviation_pub.publish(Float32(data=rgt_dev))
						self.middle_deviation_pub.publish(Float32(data=mid_dev))
						self.left_deviation_pub.publish(Float32(data=lft_dev))
						self.cell_state_pub.publish(String(data=cell_state))
						# self.get_logger().info('Publishing sensor messages')
					else:
						pass
						# self.get_logger().warn('Non valid data received, not publishing any message')
				elif self._comm_interface == 'rs232':
					# self._ser.flushInput()
					data = self._ser.read_until(bytes(RS232_FOOTER))
					# self.get_logger().info(data.hex(':'))
					if len(data)==RS232_BYTES_SIZE and data[0:5]==RS232_HEADER and data[-1:]==RS232_FOOTER:
						cell_state = str(format(int(data[35:37].hex(), base=BITS_PER_DATA), '015b'))
						rgt_dev = self.twos_complement(data[37:39].hex(),BITS_PER_DATA) * MM_PER_UNIT
						mid_dev = self.twos_complement(data[39:41].hex(),BITS_PER_DATA) * MM_PER_UNIT		
						lft_dev = self.twos_complement(data[41:43].hex(),BITS_PER_DATA) * MM_PER_UNIT
						self.right_deviation_pub.publish(Float32(data=rgt_dev))
						self.middle_deviation_pub.publish(Float32(data=mid_dev))
						self.left_deviation_pub.publish(Float32(data=lft_dev))
						self.cell_state_pub.publish(String(data=cell_state))
						# self.get_logger().info('Publishing sensor messages')
					else:
						pass
						# self.get_logger().warn('Non valid data received, not publishing any message')
				# rgt_dev_str = (str(rgt_dev)+' mm') if abs(rgt_dev)<200 else 'n/a'
				# mid_dev_str = (str(mid_dev)+' mm') if abs(mid_dev)<200 else 'n/a'
				# lft_dev_str = (str(lft_dev)+' mm') if abs(lft_dev)<200 else 'n/a'
				# self.get_logger().info('cell state = ' + str(cell_state_bin) + ' , mid_dev = ' + mid_dev_str)

	def twos_complement(self, hexstr, bits):
		value = int(hexstr, BITS_PER_DATA)
		if value & (1 << (bits-1)):
			value -= 1 << bits
		return value

	# def access_bit(self, data, num):
	#     base = int(num // 8)
	#     shift = int(num % 8)
	#     return (data[base] >> shift) & 0x1

	# def s16(self, value):
	#     return -(value & 0x8000) | (value & 0x7fff)


def main(args=None):
	rclpy.init(args=args)
	executor = MultiThreadedExecutor()
	node = TzbotMagDriver()
	executor.add_node(node)
	executor.spin()
	executor.shutdown()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()