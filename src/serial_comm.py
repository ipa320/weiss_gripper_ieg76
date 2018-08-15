#!/usr/bin/env python
# import roslib
import struct
import time
import serial
import rospy
import threading
# import diagnostic_updater
from serial import SerialException
# from std_srvs.srv import Trigger, TriggerResponse
# from weiss_gripper_ieg76.srv import *
# from sensor_msgs.msg import JointState
# from diagnostic_msgs.msg import DiagnosticStatus


def create_send_payload(command, grasping_force = 100, opening_position = 29.50, closing_position = 0.5):
	grasp_config_no = 0
	grasp_index = "00"

	if command == "open":
		send_cmd = "PDOUT=[02," + grasp_index + "]\n"
	elif command == "close":
		send_cmd = "PDOUT=[03," + grasp_index + "]\n"
	elif command == "set_grasping_force":
		grasp_index = 96 + grasp_config_no
		grasping_force_hex = hex(grasping_force)[2:]
		send_cmd = "SETPARAM(" + str(grasp_index) + ", 3, [" + grasping_force_hex + "])\n"
	elif command == "set_opening_position":
		grasp_index = 96 + grasp_config_no
		opening_position = round(opening_position, 2)
		opening_pos = int(opening_position * 100)
		opening_pos_hex = hex(opening_pos)[2:]
		opening_pos_hex_byte0 = opening_pos_hex[:-2]
		if not opening_pos_hex_byte0:
			opening_pos_hex_byte0 = hex(0)[:-2]
		opening_pos_hex_byte1 = opening_pos_hex[-2:]
		send_cmd = "SETPARAM(" + str(grasp_index) + ", 2, [" + opening_pos_hex_byte0[0:2] + "," + opening_pos_hex_byte1[0:2] + "])\n"
	elif command == "set_closing_position":
		grasp_index = 96 + grasp_config_no
		closing_position = round(closing_position, 2)
		closing_pos = int(closing_position * 100)
		closing_pos_hex = hex(closing_pos)[2:]
		closing_pos_hex_byte0 = closing_pos_hex[:-2]
		if not closing_pos_hex_byte0:
			closing_pos_hex_byte0 = hex(0)[:-2]
		closing_pos_hex_byte1 = closing_pos_hex[-2:]
		send_cmd = "SETPARAM(" + str(grasp_index) + ", 1, [" + closing_pos_hex_byte0[0:2] + "," + closing_pos_hex_byte1[0:2] + "])\n"
	else:
		cmd_dict = {"query":"ID?\n", "activate":"PDOUT=[02,00]\n", "operate":"OPERATE()\n", "reference":"PDOUT=[07,00]\n",
					"deactivate":"PDOUT=[00,00]\n", "fallback":"FALLBACK(1)\n", "mode":"MODE?\n", "restart":"RESTART()\n",
					"reset":"PDOUT=[00,00]\n", "get_vendor_name","GETPARAM(16)", "get_vendor_text","GETPARAM(17)",
					"get_product_name","GETPARAM(18)", "get_product_id","GETPARAM(19)", "get_product_text","GETPARAM(20)",
					"get_serial_no","GETPARAM(21)", "get_device_status","GETPARAM(36)", "get_detailed_device_status","GETPARAM(37)"}
		# Query if command is known
		if not(command in cmd_dict):
			rospy.logerr("Command not recognized")
			return None
		send_cmd = cmd_dict[command]

	# create format for command
	fmt = '>'
	for i in range(len(send_cmd)):
		fmt+='B'

	# create bytelist from command
	bytelist =[ord(c) for c in send_cmd]
	# create payload
	payload = struct.pack(fmt,*bytelist)
	return payload


class SerialPortComm(threading.Thread):
	def __init__(self, serial_port, serial_timeout):
		self.flags_msg_length = 21
		self.get_all_param_msg_length = 32
		self.get_positions_param_msg_length = 23
		self.get_grasping_force_param_msg_length = 20
		self.set_single_param_msg_length = 19
		self.input_data_unavailable = 0

		self.flags_observers = []
		self.current_flags_dict = {"POS":0.0, "OPEN_FLAG":0b0, "CLOSED_FLAG":0b0, "HOLDING_FLAG":0b0, "FAULT_FLAG":0b0, "IDLE_FLAG":0b0, "TEMPFAULT_FLAG":0b0, "TEMPWARN_FLAG":0b0, "MAINT_FLAG":0b0}
		self.grasp_config = {"grasp_config_no": 0, "grasping_force": 100, "closing_position": 0.5, "opening_position": 29.5, "fresh": False}

		self.serial = serial.Serial()
		self.serial_write_sync = threading.Lock()

		self.driver_shutdown = False
		self.open_serial_port(serial_port, serial_timeout)

		rospy.logdebug("get_vendor_name")
		self.send_command("get_vendor_name")
		time.sleep(0.5)
		rospy.logdebug("get_vendor_text")
		self.send_command("get_vendor_text")
		time.sleep(0.5)
		rospy.logdebug("get_product_name")
		self.send_command("get_product_name")
		time.sleep(0.5)
		rospy.logdebug("get_product_id")
		self.send_command("get_product_id")
		time.sleep(0.5)
		rospy.logdebug("get_product_text")
		self.send_command("get_product_text")
		time.sleep(0.5)
		rospy.logdebug("get_serial_no")
		self.send_command("get_serial_no")
		time.sleep(0.5)
		rospy.logdebug("get_device_status")
		self.send_command("get_device_status")
		time.sleep(0.5)
		rospy.logdebug("get_detailed_device_status")
		self.send_command("get_detailed_device_status")
		time.sleep(0.5)

		self.initialize_gripper()

		self.changing_settings = threading.Lock() # only one setting at a time
		self.setting_changed_successfully = False
		self.setting_changed = threading.Condition()

		threading.Thread.__init__(self)

	def open_serial_port(self, serial_port, serial_timeout):
		self.serial.port = serial_port
		self.serial.timeout = serial_timeout
		while not(self.driver_shutdown):
			try:
				self.serial.open()
				self.serial.flushInput()
				self.serial.flushOutput()
				time.sleep(.1)
				break
			except Exception as e:
				rospy.logerr("\terror opening serial port %s : %s", serial_port, e)
				rospy.loginfo("Retrying to open the serial port %s...", serial_port)
				time.sleep(1)

		rospy.loginfo("Serial port %s opened: %s", self.serial.port, self.serial.isOpen())

	def initialize_gripper(self):
		rospy.loginfo("(Re)initializing...")
		with self.serial_write_sync:
			try:
				rospy.logdebug("Query")
				self.send_command("query")
				time.sleep(0.5)
				self.send_command("query")
				time.sleep(0.5)
				rospy.logdebug("Fallback")
				self.send_command("fallback")
				time.sleep(0.5)
				rospy.logdebug("Mode")
				self.send_command("mode")
				time.sleep(0.5)
				rospy.logdebug("Restart")
				self.send_command("restart")
				time.sleep(0.5)
				rospy.logdebug("Operate")
				self.send_command("operate")
				time.sleep(0.5)
				rospy.logdebug("Reset flags")
				self.send_command("reset")
				time.sleep(0.5)
				self.input_data_unavailable = 0
				rospy.loginfo("Ready to receive requests.")
			except Exception as e:
				rospy.logerr("Error reading from the serial port while (re)connect: %s", e)

	def shutdown(self):
		self.driver_shutdown = True

		with self.serial_write_sync:
			try:
				rospy.logdebug("Deactivate.")
				self.send_command("deactivate")
				time.sleep(0.5)
				rospy.logdebug("Fallback.")
				self.send_command("fallback")
				time.sleep(0.5)
				if self.serial.isOpen():
					rospy.logdebug("Close port.")
					self.serial.close()
			except SerialException as e:
				rospy.logerr("Error closing the serial port: %s", e)

	def send_command(self, command, grasping_force = 100, opening_position = 29.50, closing_position = 0.5):
		if not self.serial_write_sync.locked():
			self.send_command_synced(command, grasping_force, opening_position, closing_position)
		payload = create_send_payload(command, grasping_force, opening_position, closing_position)

		self.log_debug_flags()
		try:
			self.serial.write(payload)
			rospy.logdebug("Message sent to serial port")
		except SerialException as e:
			rospy.logerr("Error writing to the serial port: %s", e)

	def send_command_synced(self, command, grasping_force = 100, opening_position = 29.50, closing_position = 0.5):
		with self.serial_write_sync:
			self.send_command(command, grasping_force, opening_position, closing_position)

	def change_setting(self, command, grasping_force = 100, opening_position = 29.50, closing_position = 0.5):
		with self.changing_settings: # only change one setting at a time
			with self.setting_changed:
				self.setting_changed_successfully = False
				self.send_command_synced(command, grasping_force, opening_position, closing_position)
				self.setting_changed.wait(3.0)
				return self.setting_changed_successfully

	def set_force(self, grasping_force = 100):
		return self.change_setting("set_grasping_force", grasping_force = grasping_force)

	def set_opening_pos(self, opening_position = 29.50):
		return self.change_setting("set_opening_position", opening_position = opening_position)

	def set_closing_pos(self, closing_position = 0.5):
		return self.change_setting("set_closing_position", closing_position = closing_position)

	def add_flags_observer(self, observer):
		self.flags_observers.append(observer)

	def log_debug_flags(self):
		rospy.logdebug("POS = %f", self.current_flags_dict["POS"])
		rospy.logdebug("IDLE_FLAG = %s", self.current_flags_dict["IDLE_FLAG"])
		rospy.logdebug("OPEN_FLAG = %s", self.current_flags_dict["OPEN_FLAG"])
		rospy.logdebug("CLOSED_FLAG = %s", self.current_flags_dict["CLOSED_FLAG"])
		rospy.logdebug("HOLDING_FLAG = %s", self.current_flags_dict["HOLDING_FLAG"])
		rospy.logdebug("FAULT_FLAG = %s", self.current_flags_dict["FAULT_FLAG"])
		rospy.logdebug("TEMPFAULT_FLAG = %s", self.current_flags_dict["TEMPFAULT_FLAG"])
		rospy.logdebug("TEMPWARN_FLAG = %s", self.current_flags_dict["TEMPWARN_FLAG"])
		rospy.logdebug("MAINT_FLAG = %s", self.current_flags_dict["MAINT_FLAG"])

	def ack_set_param(self, read_data_hexstr):
		read_data = read_data_hexstr[0:12]
		if read_data == "FIN SETPARAM":
			with self.setting_changed:
				self.setting_changed_successfully = True
				self.setting_changed.notify()


	def extract_flags(self, read_data_hexstr):
		#the data read from the serial port is @PDIN=[BYTE0,BYTE1,BYTE2,BYTE3] (see pag.20 in user manual)
		position_hexstr = read_data_hexstr[7:9] + read_data_hexstr[10:12] #remove the comma "," bewteen "BYTE0" and "BYTE1"
		self.current_flags_dict["POS"] = int(position_hexstr, 16) / float(100) #position in mm

		byte3_hexstr = read_data_hexstr[16:18]
		byte3_binary = int(byte3_hexstr, 16)
		mask = 0b1
		self.current_flags_dict["IDLE_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["OPEN_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["CLOSED_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["HOLDING_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["FAULT_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["TEMPFAULT_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["TEMPWARN_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		self.current_flags_dict["MAINT_FLAG"] = byte3_binary & mask

		for obs in self.flags_observers:
			obs.update_flags(self.current_flags_dict)

	def reconnect_serial_port(self):
		serial_port_addr = self.serial.port
		serial_timeout = self.serial.timeout
		rospy.loginfo("Reconnecting to the serial port %s from SerialPortComm...", serial_port_addr)
		if self.serial.isOpen():
			self.serial.close()
		self.serial = None
		time.sleep(2)
		self.serial = serial.Serial()

		self.open_serial_port(serial_port_addr, serial_timeout)
		self.initialize_gripper()

	def parse_incoming_data_block(self, read_data_hexstr):
		incoming_msgs = read_data_hexstr.splitlines()#returns a list of incoming messages without the line break "\n"
		for msg in incoming_msgs:
			print "IN: ", msg
			if len(msg) == self.flags_msg_length:
				self.extract_flags(msg)
			elif len(msg) == self.set_single_param_msg_length:
				self.ack_set_param(msg)

	def run(self):
		rospy.logdebug("SerialPortComm.run()")
		#read from port
		connection_errors_no = 0
		incoming_bytes_no = 0

		#first incoming msg-block contains start-up msgs and should not be parsed
		if (not self.driver_shutdown) and self.serial.isOpen():
			try:
				incoming_bytes_no = self.serial.inWaiting()

				if (incoming_bytes_no>0): #if incoming bytes are waiting to be read from the serial input buffer
					input_data = self.serial.read(self.serial.inWaiting())
					data_str = input_data.decode('ascii') #read the bytes and convert from binary array to ASCII
					rospy.logdebug("incoming_bytes_no = %d: %s", incoming_bytes_no, data_str)
			except Exception as e:
				rospy.logerr("SerialPortComm.run() - inside if statement: %s", e)

		#parse the subsequent reads
		while (not self.driver_shutdown) and self.serial.isOpen():
			try:
				incoming_bytes_no = self.serial.inWaiting()
				if (incoming_bytes_no>0): #if incoming bytes are waiting to be read from the serial input buffer
					self.input_data_unavailable = 0
					input_data = self.serial.read(self.serial.inWaiting())
					data_str = input_data.decode('ascii') #read the bytes and convert from binary array to ASCII
					if (incoming_bytes_no <> 22):
						rospy.logdebug("incoming_bytes_no = %d: %s", incoming_bytes_no, data_str)
					self.parse_incoming_data_block(data_str)
				else:
					self.input_data_unavailable+=1
					time.sleep(0.1)
					if(self.input_data_unavailable>50):
						rospy.logerr("No data received from the gripper. Unplug and replug the usb-cable.")
			except Exception as e:
				rospy.logerr("SerialPortComm.run() - inside while statement: %s", e)
				connection_errors_no += 1
				if(connection_errors_no > 5):
					connection_errors_no = 0 #reset the counter
					self.reconnect_serial_port()
		rospy.logdebug("SerialPortComm_thread done.")
