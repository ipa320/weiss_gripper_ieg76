#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from weiss_gripper_ieg76.srv import Move, MoveResponse, SetForce, SetForceResponse
from weiss_gripper_ieg76.serial_comm import SerialPortComm
from weiss_gripper_ieg76.driver_logic import DriverLogic
from weiss_gripper_ieg76.state_publisher import StatesPublisher


class Driver(object):
	def __init__(self):
		serial_port_addr = rospy.get_param("~serial_port_address", '/dev/ttyACM0')
		self.serial_port_comm = SerialPortComm(serial_port_addr, serial_timeout=0)

		self.driver_logic = DriverLogic(self.serial_port_comm)

		self.states_publisher_thread = StatesPublisher(0.8, self.serial_port_comm)
		rospy.on_shutdown(self.shutdown_handler)

	def set_max_pos(self, gripper_info):
		product_id = gripper_info["product_id"]
		if product_id == "CRG 200-085":
			self._max_pos = 85
		elif product_id == "CRG 30-050":
			self._max_pos = 50
		elif product_id == "IEG 76-030":
			self._max_pos = 30
		elif product_id == "IEG 55-020":
			self._max_pos = 20
		else:
			raise RuntimeError("Gripper type '{}' not yet supported!".format(product_id))

	def log_reply(self, reply):
		if reply.success:
			rospy.loginfo(reply.message)
		else:
			rospy.logerr(reply.message)

	def check_position(self, pos):
		return 0 <= pos <= self._max_pos

	def check_force(self, force):
		return 0 <= force <= 100

	def handle_reference(self, req):
		rospy.loginfo("Referencing")
		reply = TriggerResponse()
		self.driver_logic.service_called(transition="do_reference", params=req, trigger_response=reply)
		reply.message = 'Referencing ' + reply.message
		self.log_reply(reply)
		return reply

	def relative_to_absolute(self, req):
		if req.relative:
			if self.driver_logic.gripper_pos is None:
				raise ValueError("Relative position demanded, but gripper position was not received yet.")
			req.position = self.driver_logic.gripper_pos + req.position

	def handle_open(self, req):
		rospy.loginfo("Opening")
		reply = MoveResponse()
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Opening failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.driver_logic.service_called(transition="do_open", params=req, trigger_response=reply)
			reply.message = 'Opening ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_close(self, req):
		rospy.loginfo("Closing")
		reply = MoveResponse()
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Closing failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.driver_logic.service_called(transition="do_close", params=req, trigger_response=reply)
			reply.message = 'Closing ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_grasp(self, req):
		rospy.loginfo("Grasping")
		reply = MoveResponse()
		self.relative_to_absolute(req)
		if not self.check_position(req.position):
			reply.success = False
			reply.message = 'Grasping failed. Position must be 0.0(mm) <= position <= {}.0(mm).'.format(self._max_pos)
		else:
			self.driver_logic.service_called(transition="do_grasp", params=req, trigger_response=reply)
			reply.message = 'Grasping ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_set_force(self, req):
		rospy.loginfo("Set force")
		reply = SetForceResponse()
		if not self.check_force(req.grasping_force):
			reply.success = False
			reply.message = 'Force must be 0(%) <= force <= 100(%).'
		else:
			reply.success = self.serial_port_comm.set_force(req.grasping_force)
		if reply.success:
			reply.message = 'Set force successful.'
		else:
			reply.message = 'Set force failed. ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_ack(self, req):
		rospy.loginfo("Ack error")
		reply = TriggerResponse()
		self.driver_logic.service_called(transition="do_ack", params=req, trigger_response=reply)
		reply.message = 'Ack error ' + reply.message
		self.log_reply(reply)
		return reply

	def handle_deactivate(self, req):
		rospy.loginfo("Deactivate gripper")
		reply = TriggerResponse()
		self.driver_logic.service_called(transition="do_deactivate", params=req, trigger_response=reply)
		reply.message = 'Deactivating ' + reply.message
		self.log_reply(reply)
		return reply

	def shutdown_handler(self):
		self.states_publisher_thread.shutdown()
		self.serial_port_comm.shutdown()
		rospy.loginfo("Gracefully shutting down the driver...")

	def run(self):
		self.serial_port_comm.daemon = True
		self.states_publisher_thread.daemon = True

		rospy.logdebug("Starting threads...")
		self.serial_port_comm.start()
		self.states_publisher_thread.start()
		rospy.logdebug("Threads started.")

		rospy.loginfo('Receiving info about gripper')
		self.serial_port_comm.get_gripper_info()
		print(self.serial_port_comm.gripper_info)
		serial_no = rospy.get_param("~serial_no", None)
		if not serial_no is None and serial_no != int(self.serial_port_comm.gripper_info['serial_no']):
			rospy.logerr('Serial no is {}, but requested is {}'.format(self.serial_port_comm.gripper_info['serial_no'], serial_no))
			return

		# update parameters based on gripper_info
		self.set_max_pos(self.serial_port_comm.gripper_info)
		self.states_publisher_thread.set_gripper_info(self.serial_port_comm.gripper_info)

		grasp_force = rospy.get_param("~grasping_force", 100)
		rospy.loginfo('Setting force to {}%...'.format(grasp_force))
		while True:
			if self.serial_port_comm.set_force(grasp_force):
				break
		rospy.loginfo('Force set.')

		serv_ref = rospy.Service('~reference', Trigger, self.handle_reference)
		serv_ref = rospy.Service('~open', Move, self.handle_open)
		serv_ref = rospy.Service('~close', Move, self.handle_close)
		serv_ref = rospy.Service('~grasp', Move, self.handle_grasp)
		serv_ref = rospy.Service('~ack', Trigger, self.handle_ack)
		serv_ref = rospy.Service('~set_force', SetForce, self.handle_set_force)
		serv_ref = rospy.Service('~deactivate', Trigger, self.handle_deactivate)

		rospy.loginfo("Ready to receive requests.")

		rospy.spin()


if __name__ == "__main__":
	# rospy.init_node('ieg_driver', log_level=rospy.DEBUG)
	rospy.init_node('ieg_driver')

	driver = Driver()
	driver.run()
