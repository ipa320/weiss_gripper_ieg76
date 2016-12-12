#!/usr/bin/env python
import roslib
import struct
import time
import serial
import rospy
import threading
import diagnostic_updater
from serial import SerialException
from std_srvs.srv import Trigger, TriggerResponse
from weiss_gripper_ieg76.srv import ConfigTrigger, ConfigTriggerResponse
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus 

ser = serial.Serial()
serial_port_lock = threading.Lock()

flags_lock = threading.Lock()
fresh_flags_cond_var = threading.Condition()#this uses the lock created by default
update_flags_cond_var = threading.Condition()
reference_cond_var = threading.Condition(flags_lock)
jaws_closed_cond_var = threading.Condition(flags_lock)
jaws_opened_cond_var = threading.Condition(flags_lock)
object_grasped_cond_var = threading.Condition(flags_lock)
fault_cond_var = threading.Condition(flags_lock)

current_flags_dict = {"POS":0.0, "OPEN_FLAG":0b0, "CLOSED_FLAG":0b0, "HOLDING_FLAG":0b0, "FAULT_FLAG":0b0, "IDLE_FLAG":0b0, "TEMPFAULT_FLAG":0b0, "TEMPWARN_FLAG":0b0, "MAINT_FLAG":0b0}
old_flags_dict = {"POS":0.0, "OPEN_FLAG":0b0, "CLOSED_FLAG":0b0, "HOLDING_FLAG":0b0, "FAULT_FLAG":0b0, "IDLE_FLAG":0b0, "TEMPFAULT_FLAG":0b0, "TEMPWARN_FLAG":0b0, "MAINT_FLAG":0b0}

shutdown_driver = False

def log_debug_flags():
	rospy.logdebug("POS = %f", current_flags_dict["POS"])
	rospy.logdebug("IDLE_FLAG = %s", current_flags_dict["IDLE_FLAG"])
	rospy.logdebug("OPEN_FLAG = %s", current_flags_dict["OPEN_FLAG"])
	rospy.logdebug("CLOSED_FLAG = %s", current_flags_dict["CLOSED_FLAG"])
	rospy.logdebug("HOLDING_FLAG = %s", current_flags_dict["HOLDING_FLAG"])
	rospy.logdebug("FAULT_FLAG = %s", current_flags_dict["FAULT_FLAG"])
	rospy.logdebug("TEMPFAULT_FLAG = %s", current_flags_dict["TEMPFAULT_FLAG"])
	rospy.logdebug("TEMPWARN_FLAG = %s", current_flags_dict["TEMPWARN_FLAG"])
	rospy.logdebug("MAINT_FLAG = %s", current_flags_dict["MAINT_FLAG"])

def create_send_payload(command, grasp_config_no = 0):
	if grasp_config_no == 0:
		grasp_index = "00"
	elif grasp_config_no == 1:
		grasp_index = "01"
	elif grasp_config_no == 2:
		grasp_index = "02"
	elif grasp_config_no == 2:
		grasp_index = "03"
	else:
		raise ValueError("Unknown grasp config no " + str(grasp_config_no) +" given. Valid grasp configuration numbers are 0, 1, 2, 3.")

	if command == "open":
		send_cmd = "PDOUT=[02," + grasp_index + "]\n"
	elif command == "close":
		send_cmd = "PDOUT=[03," + grasp_index + "]\n"
	else:
		cmd_dict = {"query":"ID?\n", "activate":"PDOUT=[02,00]\n", "operate":"OPERATE()\n", "reference":"PDOUT=[07,00]\n", "deactivate":"PDOUT=[00,00]\n", "fallback":"FALLBACK(1)\n", "mode":"MODE?\n", "restart":"RESTART()\n", "reset":"PDOUT=[00,00]\n"}
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


class serial_port_reader(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

	def extract_info(self, read_data_hexstr):
		fresh_flags_cond_var.acquire()
		#rospy.logdebug("serial_port_reader inside extract_info")
		global current_flags_dict
		global old_flags_dict
		#the data read from the serial port is @PDIN=[BYTE0,BYTE1,BYTE2,BYTE3] (see pag.20 in user manual)
		position_hexstr = read_data_hexstr[7:9] + read_data_hexstr[10:12] #remove the comma "," bewteen "BYTE0" and "BYTE1"
		current_flags_dict["POS"] = int(position_hexstr, 16) / float(100) #position in mm

		byte3_hexstr = read_data_hexstr[16:18]
		byte3_binary = int(byte3_hexstr, 16)
		mask = 0b1
		#rospy.logdebug("serial_port_reader reference_cond_var.acquire()")
		reference_cond_var.acquire()
		try:
			#rospy.logdebug("serial_port_reader has acquired reference_cond_var")
			old_flags_dict["IDLE_FLAG"] = current_flags_dict["IDLE_FLAG"]
			current_flags_dict["IDLE_FLAG"] = byte3_binary & mask
			if old_flags_dict["IDLE_FLAG"] == 0 and current_flags_dict["IDLE_FLAG"] == 1:
				# signal the event
				reference_cond_var.notify()
				rospy.logdebug("Reference event triggered")
		finally:
			#rospy.logdebug("serial_port_reader releasing reference_cond_var")	
			reference_cond_var.release()	

		byte3_binary = byte3_binary >> 1
		jaws_opened_cond_var.acquire()
		try:
			old_flags_dict["OPEN_FLAG"] = current_flags_dict["OPEN_FLAG"]
			current_flags_dict["OPEN_FLAG"] = byte3_binary & mask
			if old_flags_dict["OPEN_FLAG"] == 0 and current_flags_dict["OPEN_FLAG"] == 1:
				#the transition from jaws not opened to jaws opened has occured. Signal this event.
				jaws_opened_cond_var.notify()
				rospy.logdebug("Open event triggered")
		finally:
			jaws_opened_cond_var.release()

		byte3_binary = byte3_binary >> 1
		jaws_closed_cond_var.acquire()
		try:
			old_flags_dict["CLOSED_FLAG"] = current_flags_dict["CLOSED_FLAG"]
			current_flags_dict["CLOSED_FLAG"] = byte3_binary & mask
			if old_flags_dict["CLOSED_FLAG"] == 0 and current_flags_dict["CLOSED_FLAG"] == 1:
				#the transition from jaws not closed to jaws closed has occured. Signal this event.
				jaws_closed_cond_var.notify()
				rospy.logdebug("Close event triggered")
		finally:
			jaws_closed_cond_var.release()

		byte3_binary = byte3_binary >> 1
		object_grasped_cond_var.acquire()
		try:
			old_flags_dict["HOLDING_FLAG"] = current_flags_dict["HOLDING_FLAG"]
			current_flags_dict["HOLDING_FLAG"] = byte3_binary & mask
			if old_flags_dict["HOLDING_FLAG"] == 0 and current_flags_dict["HOLDING_FLAG"] == 1:
				#the transition from not holding/grasping an object to holding/grasping an object has occured. Signal this event.
				object_grasped_cond_var.notify()
				rospy.logdebug("Grasp event triggered")
		finally:
			object_grasped_cond_var.release()

		byte3_binary = byte3_binary >> 1
		fault_cond_var.acquire()
		try:
			current_flags_dict["FAULT_FLAG"] = byte3_binary & mask
		finally:
			fault_cond_var.release()

		byte3_binary = byte3_binary >> 1
		current_flags_dict["TEMPFAULT_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		current_flags_dict["TEMPWARN_FLAG"] = byte3_binary & mask

		byte3_binary = byte3_binary >> 1
		current_flags_dict["MAINT_FLAG"] = byte3_binary & mask

		fresh_flags_cond_var.notify()# signal to states_publisher that new data is available for publishing
		fresh_flags_cond_var.release()

	def reconnect_serial_port(self):
		global ser
		serial_port_addr = ser.port
		rospy.loginfo("Reconnecting to the serial port %s from serial_port_reader...", serial_port_addr)
		if ser.isOpen(): 
			ser.close()
		ser = None
		time.sleep(2)
		ser = serial.Serial()
		ser.port = serial_port_addr
		ser.timeout = 0
		is_serial_port_opened = False

		while not is_serial_port_opened:
			try: 
				ser.open()
				ser.flushInput()
				ser.flushOutput()
				is_serial_port_opened = True
			except Exception as e:
				is_serial_port_opened = False
				rospy.logerr("Error opening serial port %s: %s", serial_port_addr, e)
				rospy.loginfo("Retrying to open the serial port %s...", serial_port_addr)
				time.sleep(1)
		rospy.loginfo("Serial port opened: %s", is_serial_port_opened)

		rospy.loginfo("Query")
		payload = create_send_payload("query")
		try:
			ser.write(payload)
			time.sleep(0.5)
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Mode")
			payload = create_send_payload("mode")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Restart")
			payload = create_send_payload("restart")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Operate")
			payload = create_send_payload("operate")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Reset flags")
			payload = create_send_payload("reset")
			ser.write(payload)
			time.sleep(0.5)
		except Exception as e:
			rospy.logerr("Error reading from the serial port while reconnect: %s", e)

	def run(self):
		#read from port
		connection_errors_no = 0
		incoming_bytes_no = 0
		#rospy.logdebug("serial_port_reader.run() serial_port_status = %s", ser.isOpen())
		while (not shutdown_driver) and ser.isOpen():
			try:
				incoming_bytes_no = ser.inWaiting()
				
				if (incoming_bytes_no>0): #if incoming bytes are waiting to be read from the serial input buffer
					input_data = ser.read(ser.inWaiting())
					data_str = input_data.decode('ascii') #read the bytes and convert from binary array to ASCII
					
					if incoming_bytes_no == 22:
						self.extract_info(data_str)
					else:
						rospy.logdebug("incoming_bytes_no = %d: %s",incoming_bytes_no, data_str)
			except Exception as e:
				rospy.logerr("serial_port_reader.run(): %s", e)
				connection_errors_no += 1
				if(connection_errors_no > 5):
					connection_errors_no = 0 #reset the counter
					self.reconnect_serial_port()
		rospy.logdebug("serial_port_reader_thread done.")

class states_publisher(threading.Thread):
	def __init__(self, loop_time):
		threading.Thread.__init__(self)
		self.loop_time = loop_time
		self.joint_state_msg = JointState()
		self.joint_state_msg.name = []
		self.joint_state_msg.name.append("gripper_claws")
		self.joint_states_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.updater = diagnostic_updater.Updater()
		self.updater.setHardwareID("Weiss Robotics Gripper IEG 76-030 V1.02 SN 000106")
		self.updater.add("Position and flags updater", self.produce_diagnostics)
		freq_bounds = {'min':0.5, 'max':2}
		# It publishes the messages and simultaneously makes diagnostics for the topic "joint_states" using a FrequencyStatus and TimeStampStatus
		self.pub_freq_time_diag = diagnostic_updater.DiagnosedPublisher(self.joint_states_publisher, self.updater, diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10), diagnostic_updater.TimeStampStatusParam())

	def run(self):
		global fresh_flags_cond_var
		rospy.logdebug("states_publisher.run()")
		while not shutdown_driver:
			fresh_flags_cond_var.acquire()
			fresh_flags_cond_var.wait()
			self.updater.update()
			rospy.logdebug("states_publisher publishing states...")
			self.publish_states()
			rospy.logdebug("states_publisher states published.")
			#self.updater.force_update()
			fresh_flags_cond_var.release()
			rospy.sleep(self.loop_time)
		rospy.logdebug("states_publisher_thread done.")

	def produce_diagnostics(self, stat):
		if current_flags_dict["FAULT_FLAG"] == True:
			stat.summary(DiagnosticStatus.ERROR, "The fault bit of the gripper is 1.")
		else:
			stat.summary(DiagnosticStatus.OK, "The fault bit of the gripper is 0.")
		stat.add("Position", current_flags_dict["POS"])
		stat.add("Idle Flag", current_flags_dict["IDLE_FLAG"])
		stat.add("Open Flag", current_flags_dict["OPEN_FLAG"])
		stat.add("Closed Flag", current_flags_dict["CLOSED_FLAG"])
		stat.add("Holding Flag", current_flags_dict["HOLDING_FLAG"])
		stat.add("Error Flag", current_flags_dict["FAULT_FLAG"])
		stat.add("Temperature Error Flag", current_flags_dict["TEMPFAULT_FLAG"])
		stat.add("Temperature Warning Flag", current_flags_dict["TEMPWARN_FLAG"])
		stat.add("Maintenance Flag", current_flags_dict["MAINT_FLAG"])
		return stat

	def publish_states(self):
		self.joint_state_msg.header.stamp = rospy.Time.now()
		self.joint_state_msg.position = []
		self.joint_state_msg.position.append(current_flags_dict["POS"])
		try:
			self.pub_freq_time_diag.publish(self.joint_state_msg)
		except:
			rospy.logerr("\nClosed topics.")

class driver(object):
	def __init__(self):
		global ser
		#rospy.init_node('driver_node', log_level=rospy.DEBUG)
		rospy.init_node('driver_node')
		rospy.on_shutdown(self.shutdown_handler)
		serial_port_addr = rospy.get_param("~serial_port_address", '/dev/ttyACM0')
		ser.port = serial_port_addr
		ser.timeout = 0
		is_serial_port_opened = False
		while not(shutdown_driver) and not(is_serial_port_opened):
			try: 
				ser.open()
				ser.flushInput()
				ser.flushOutput()
				is_serial_port_opened = True
			except Exception as e:
				is_serial_port_opened = False
				rospy.logerr("\terror opening serial port %s : %s", serial_port_addr, e)
				rospy.loginfo("Retrying to open the serial port %s...", serial_port_addr)
				time.sleep(1)
		
		rospy.loginfo("Serial port %s opened: %s", ser.port, ser.isOpen())

		self.initialize_gripper()

		serv_ref = rospy.Service('reference', Trigger, self.handle_reference)
		serv_open = rospy.Service('open_jaws', ConfigTrigger, self.handle_open_jaws)
		serv_close = rospy.Service('close_jaws', ConfigTrigger, self.handle_close_jaws)
		serv_grasp = rospy.Service('grasp_object', ConfigTrigger, self.handle_grasp_object)
		serv_ack_error = rospy.Service('ack_error', Trigger, self.handle_ack_error)
		serv_ack_ref_error = rospy.Service('ack_ref_error', Trigger, self.handle_ack_ref_error)

		self.serial_port_reader_thread = serial_port_reader()
		self.states_publisher_thread = states_publisher(0.8)


		rospy.loginfo("Ready to receive requests.")

	def initialize_gripper(self):
		rospy.loginfo("Query")
		payload = create_send_payload("query")
		serial_port_lock.acquire()
		try:
			ser.write(payload)
			time.sleep(0.5)
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Mode")
			payload = create_send_payload("mode")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Restart")
			payload = create_send_payload("restart")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Operate")
			payload = create_send_payload("operate")
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Reset flags")
			payload = create_send_payload("reset")
			ser.write(payload)
			time.sleep(0.5)
		
		except Exception as e:
			rospy.logerr("initialize_gripper: %s", e)
		finally:
			serial_port_lock.release()

	def check_if_referenced(self):
		#when the gripper is not referenced all flags are 0
		gripper_referenced = False
		
		if (current_flags_dict["IDLE_FLAG"] == 1 or current_flags_dict["OPEN_FLAG"] == 1 or 
				current_flags_dict["CLOSED_FLAG"] == 1 or current_flags_dict["HOLDING_FLAG"] == 1 or
				current_flags_dict["FAULT_FLAG"] == 1 or current_flags_dict["TEMPFAULT_FLAG"] == 1 or
				current_flags_dict["TEMPWARN_FLAG"] == 1 or current_flags_dict["MAINT_FLAG"] == 1):
			gripper_referenced = True

		return gripper_referenced

	def handle_reference(self, req):
		rospy.loginfo("Referencing")
		payload = create_send_payload("reference")
		reply = TriggerResponse()

		reference_cond_var.acquire()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				reference_cond_var.wait(timeout=3.0)
				if current_flags_dict["IDLE_FLAG"]:
					rospy.loginfo("Gripper referenced.")
					reply.success = True
					reply.message = "Gripper referenced."
				else:
					rospy.logerr("Failed to reference the gripper.")
					reply.success = False
					reply.message = "Failed to reference the gripper."
		except Exception as e:
			rospy.logerr("driver.handle_reference(): %s", e)
		finally:
			log_debug_flags()
			reference_cond_var.release()

		return reply

	def handle_open_jaws(self, req):
		rospy.loginfo("Opening the jaws.")
		try:
			reply = ConfigTriggerResponse()
			payload = create_send_payload("open", req.grasp_config_no)
		except ValueError as e:
			rospy.logerr("driver.handle_open_jaws(): %s", e)
			reply.success = False
			reply.message = "Failed to open the jaws because the invalid grasp configuration number " + str(req.grasp_config_no) + " was given. Valid grasp configuration numbers are 0, 1, 2, 3."
			return reply

		jaws_opened_cond_var.acquire()
		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				rospy.logdebug("Message sent to serial port")
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				rospy.logdebug("After sending the message to the serial port")
				serial_port_lock.release()	
				
			jaws_opened_cond_var.wait(timeout=3.0)
			if current_flags_dict["OPEN_FLAG"]:
				rospy.loginfo("Jaws opened using grasp config %d.", req.grasp_config_no)
				reply.success = True
				reply.message = "Jaws opened using grasp config " + str(req.grasp_config_no) + "."
			else:
				rospy.logerr("Failed to open the jaws using grasp config %d.", req.grasp_config_no)
				reply.success = False
				reply.message = "Failed to open the jaws using grasp config " + str(req.grasp_config_no) + "."
				gripper_referenced = self.check_if_referenced()
				if not gripper_referenced:
					rospy.logwarn("Reference the gripper before usage.")
					reply.message = reply.message + " Reference the gripper before usage."
				if current_flags_dict["HOLDING_FLAG"]:
					rospy.logwarn("Remove the object which is blocking the claws from opening completely and try again.")
					reply.message = reply.message + " Remove the object which is blocking the claws from opening completely and try again."
		except Exception as e:
			rospy.logerr("driver.handle_open_jaws(): %s", e)				
		finally:
			log_debug_flags()
			jaws_opened_cond_var.release()		

		return reply

	def handle_close_jaws(self, req):
		rospy.loginfo("completely closing the jaws.")
		try:
			reply = ConfigTriggerResponse()
			payload = create_send_payload("close", req.grasp_config_no)
		except ValueError as e:
			rospy.logerr("driver.handle_close_jaws(): %s", e)
			reply.success = False
			reply.message = "Failed to close the jaws because the invalid grasp configuration number " + str(req.grasp_config_no) + " was given. Valid grasp configuration numbers are 0, 1, 2, 3."
			return reply

		jaws_closed_cond_var.acquire()
		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("driver.handle_close_jaws() while trying to write on the serial port: %s", e)	
			finally:
				serial_port_lock.release()	

			jaws_closed_cond_var.wait(timeout=3.0)#always returns None for python 2
			if current_flags_dict["CLOSED_FLAG"]:
				rospy.loginfo("Jaws completely closed using grasp config %d.", req.grasp_config_no)
				reply.success = True
				reply.message = "Jaws completely closed using grasp config " + str(req.grasp_config_no) + "."
			else:
				rospy.logerr("Failed to completely close the jaws using grasp config %d.", req.grasp_config_no)
				reply.success = False
				reply.message = "Failed to completely close the jaws using grasp config " + str(req.grasp_config_no) + "."
				gripper_referenced = self.check_if_referenced()
				rospy.loginfo("gripper_referenced = %d", gripper_referenced)
				if not gripper_referenced:
					rospy.logwarn("Reference the gripper before usage.")
					reply.message = reply.message + " Reference the gripper before usage."

		except Exception as e:
			rospy.logerr("driver.handle_close_jaws(): %s", e)
		finally:
			log_debug_flags()
			jaws_closed_cond_var.release()

		return reply

	def handle_grasp_object(self, req):
		rospy.loginfo("Grasping object.")
		try:
			reply = ConfigTriggerResponse()
			payload = create_send_payload("close", req.grasp_config_no)
		except ValueError as e:
			rospy.logerr("driver.handle_grasp_object(): %s", e)
			reply.success = False
			reply.message = "Failed to grasp an object because the invalid grasp configuration number " + str(req.grasp_config_no) + " was given. Valid grasp configuration numbers are 0, 1, 2, 3."
			return reply

		object_grasped_cond_var.acquire()
		try:
			log_debug_flags()

			try:
				serial_port_lock.acquire()
				ser.write(payload)
			except SerialException as e:
				rospy.logerr("Error while writing on the serial port: %s", e)	
			finally:
				serial_port_lock.release()		

				object_grasped_cond_var.wait(timeout=3.0)
				if current_flags_dict["HOLDING_FLAG"]:
					rospy.loginfo("Object grasped using grasp config %d.", req.grasp_config_no)
					reply.success = True
					reply.message = "Object grasped using grasp config " + str(req.grasp_config_no) + "."
				else:
					rospy.logerr("Timed out while trying to grasp an object using grasp config %d.", req.grasp_config_no)
					reply.success = False
					reply.message = "Timed out while trying to grasp an object using grasp config " + str(req.grasp_config_no) + "."
					gripper_referenced = self.check_if_referenced()
					if not gripper_referenced:
						rospy.logwarn("Reference the gripper before usage.")
						reply.message = reply.message + " Reference the gripper before usage."
					if current_flags_dict["CLOSED_FLAG"]: 
						rospy.logwarn("No object to grasp.")
						reply.message = reply.message + " No object to grasp."
		except Exception as e:
			rospy.logerr("driver.handle_grasp_jaws(): %s", e)				
		finally:
			log_debug_flags()
			object_grasped_cond_var.release()		

		return reply

	def handle_ack_error(self, req):
		rospy.loginfo("Acknowledging error")
		payload = create_send_payload("activate")
		reply = TriggerResponse()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				time.sleep(0.5)
				payload = create_send_payload("deactivate")
				ser.write(payload)
				time.sleep(1)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				fault_cond_var.acquire()
				if not current_flags_dict["FAULT_FLAG"]:
					rospy.loginfo("Error acknowledged.")
					reply.success = True
					reply.message = "Error acknowledged"
				else:
					rospy.logerr("Failed to acknowledge the error.")
					reply.success = False
					reply.message = "Failed to acknowledge the error"
		except Exception as e:
			rospy.logerr("driver.handle_ack_error(): %s", e)
		finally:
			log_debug_flags()
			fault_cond_var.release()

		return reply

	def handle_ack_ref_error(self, req):
		rospy.loginfo("Acknowledging reference error")
		payload = create_send_payload("deactivate")
		reply = TriggerResponse()

		try:
			log_debug_flags()
			try:
				serial_port_lock.acquire()
				ser.write(payload)
				time.sleep(1)
			except SerialException as e:
				rospy.logerr("Error writing to the serial port: %s", e)
			finally:
				serial_port_lock.release()
				
				fault_cond_var.acquire()
				if not current_flags_dict["FAULT_FLAG"]:
					rospy.loginfo("Reference error acknowledged.")
					reply.success = True
					reply.message = "Reference error acknowledged"
				else:
					rospy.logerr("Failed to acknowledge the reference error.")
					reply.success = False
					reply.message = "Failed to acknowledge the reference error"
		except Exception as e:
			rospy.logerr("driver.handle_ack_ref_error(): %s", e)
		finally:
			log_debug_flags()
			fault_cond_var.release()

		return reply

	def shutdown_handler(self):
		global shutdown_driver
		global ser
		shutdown_driver = True
		payload = create_send_payload("deactivate")

		try:
			rospy.loginfo("Deactivate.")
			serial_port_lock.acquire()
			ser.write(payload)
			time.sleep(0.5)
			rospy.loginfo("Fallback.")
			payload = create_send_payload("fallback")
			ser.write(payload)
			time.sleep(0.5)
			if ser.isOpen():
				rospy.loginfo("Close port.")
				ser.close()
		except SerialException as e:
			rospy.logerr("Error closing the serial port: %s", e)
		finally:
			serial_port_lock.release()
		rospy.loginfo("Gracefully shutting down the driver...")

	def run(self):
		self.serial_port_reader_thread.daemon = True
		self.states_publisher_thread.daemon = True

		rospy.loginfo("Starting threads...")
		self.serial_port_reader_thread.start()
		self.states_publisher_thread.start()
		rospy.loginfo("Threads started.")
		
		rospy.spin()

if __name__ == "__main__":
	driver = driver()
	driver.run()