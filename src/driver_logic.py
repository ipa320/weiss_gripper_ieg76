#!/usr/bin/env python
# import roslib
# import struct
# import time
# import serial
import rospy
import numpy as np
import threading
# import diagnostic_updater
# from serial import SerialException
# from std_srvs.srv import Trigger, TriggerResponse
# from weiss_gripper_ieg76.srv import *
# from sensor_msgs.msg import JointState
# from diagnostic_msgs.msg import DiagnosticStatus
import transitions
from transitions.extensions import LockedHierarchicalMachine as Machine


class DriverLogic(object):
	states = [
		'not_initialized', 'fault', 'ref_fault', 'other_fault',

		{'name': 'st', 'children':[
		  'inactive', 'open', 'closed', 'holding']
		},

		{'name': 'op', 'on_exit': 'operation_finished', 'children':[
			{'name': 'deactivating_before_activate', 'on_enter': 'exec_deactivating'},
			{'name': 'activating', 'on_enter': 'exec_activating'},
			{'name': 'deactivating', 'on_enter': 'exec_deactivating'},
			{'name': 'referencing', 'on_enter': 'exec_referencing'},
			{'name': 'closing_before_opening', 'on_enter': 'exec_closing_before_opening'},
	  		{'name': 'opening', 'on_enter': 'exec_opening'},
	  		{'name': 'wait_for_opening'},
			{'name': 'opening_before_close', 'on_enter': 'exec_opening_before_closing'},
	  		{'name': 'closing', 'on_enter': 'exec_closing'},
			{'name': 'opening_before_grasp', 'on_enter': 'exec_opening_before_closing'},
	  		{'name': 'grasping', 'on_enter': 'exec_grasping'} ]
		}
	]

	transitions = [
		# uninitialized, fault, ref_fault and other_fault state
		['on_inactive', ['not_initialized', 'fault', 'ref_fault', 'other_fault'], 'st_inactive'],
		['on_open', ['not_initialized', 'fault', 'ref_fault', 'other_fault'], 'st_open'],
		['on_closed', ['not_initialized', 'fault', 'ref_fault', 'other_fault'], 'st_closed'],
		['on_holding', ['not_initialized', 'fault', 'ref_fault', 'other_fault'], 'st_holding'],
		['on_fault', ['not_initialized', 'other_fault'], 'fault'],
		['on_other_fault', ['not_initialized', 'fault', 'ref_fault'], 'other_fault'],
		['on_not_initialized', ['fault', 'ref_fault', 'other_fault'], 'not_initialized'],

		['do_reference', 'not_initialized', 'op_referencing'],
		['do_ack', 'fault', 'op_deactivating_before_activate'],
		['do_ref_ack', 'ref_fault', 'op_deactivating'],

		# inactive state
		['do_reference', 'st_inactive', 'op_referencing'],
		['do_open', 'st_inactive', 'op_opening'],
		['do_close', 'st_inactive', 'op_closing'],
		{ 'trigger': 'do_grasp', 'source': 'st_inactive', 'dest': 'op_grasping', 'conditions': 'can_grasp'},

		# open state
		['do_reference', 'st_open', 'op_referencing'],
		['do_open', 'st_open', 'op_closing_before_opening'],
		['do_close', 'st_open', 'op_closing'],
		{ 'trigger': 'do_grasp', 'source': 'st_open', 'dest': 'op_grasping', 'conditions': 'can_grasp'},

		# closed state
		['do_reference', 'st_closed', 'op_referencing'],
		['do_open', 'st_closed', 'op_opening'],
		['do_close', 'st_closed', 'op_opening_before_close'],
		{ 'trigger': 'do_grasp', 'source': 'st_closed', 'dest': 'op_opening_before_grasp', 'conditions': 'can_grasp'},

		# holding state
		['do_reference', 'st_holding', 'op_referencing'],
		{ 'trigger': 'do_open', 'source': 'st_holding', 'dest': 'op_opening', 'conditions': 'can_move_while_holding'},
		{ 'trigger': 'do_close', 'source': 'st_holding', 'dest': 'op_opening_before_close', 'conditions': 'can_move_while_holding'},

		# deactivating_before_activate state (part of do_ack)
		['on_inactive', 'op_deactivating_before_activate', 'op_activating'],

		# activating state (part of do_ack)
		{ 'trigger': 'on_open', 'source': 'op_activating', 'dest': 'st_open', 'before': 'operation_successful'},

		# deactivating state (part of do_ref_ack)
		{ 'trigger': 'on_not_initialized', 'source': 'op_deactivating', 'dest': 'not_initialized', 'before': 'operation_successful'},

		# referencing state
		{ 'trigger': 'on_inactive', 'source': 'op_referencing', 'dest': 'st_inactive', 'before': 'operation_successful'},
		{ 'trigger': 'on_fault', 'source': 'op_referencing', 'dest': 'ref_fault', 'before': 'unexpected_state_change'},

		# opening state
		{ 'trigger': 'on_open', 'source': 'op_opening', 'dest': 'st_open', 'before': 'operation_successful'},
		{ 'trigger': 'on_closed', 'source': 'op_opening', 'dest': 'op_wait_for_opening'},
		{ 'trigger': 'on_holding', 'source': 'op_opening', 'dest': 'st_holding', 'before': 'claws_blocked'}, # opening failed because blocked by object

		# closing state
		{ 'trigger': 'on_closed', 'source': 'op_closing', 'dest': 'st_closed', 'before': 'operation_successful'},
		{ 'trigger': 'on_holding', 'source': 'op_closing', 'dest': 'st_holding', 'before': 'claws_blocked'}, # failed because blocked by object

		# grasping state
		{ 'trigger': 'on_holding', 'source': 'op_grasping', 'dest': 'st_holding', 'before': 'operation_successful'},
		{ 'trigger': 'on_closed', 'source': 'op_grasping', 'dest': 'st_closed', 'before': 'no_object'}, # failed because no object to grasp

		# wait_for_opening state
		{ 'trigger': 'on_open', 'source': 'op_wait_for_opening', 'dest': 'st_open', 'before': 'operation_successful'},

		# closing_before_opening state
		['on_closed', 'op_closing_before_opening', 'op_opening'],
		{ 'trigger': 'on_holding', 'source': 'op_closing_before_opening', 'dest': 'st_holding', 'before': 'claws_blocked'}, # failed because blocked by object

		# opening_before_closing state
		['on_open', 'op_opening_before_close', 'op_closing'],
		{ 'trigger': 'on_holding', 'source': 'op_opening_before_close', 'dest': 'st_holding', 'before': 'claws_blocked'}, # failed because blocked by object

		['on_open', 'op_opening_before_grasp', 'op_grasping'],

		# default transitions for unexpected state changes
		{ 'trigger': 'on_inactive', 'source': '*', 'dest': 'st_inactive', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_open', 'source': '*', 'dest': 'st_open', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_closed', 'source': '*', 'dest': 'st_closed', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_holding', 'source': '*', 'dest': 'st_holding', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_fault', 'source': '*', 'dest': 'fault', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_other_fault', 'source': '*', 'dest': 'other_fault', 'before': 'unexpected_state_change'},
		{ 'trigger': 'on_not_initialized', 'source': '*', 'dest': 'not_initialized', 'before': 'unexpected_state_change'}
	]

	def __init__(self, serial_port_comm):
		self.state_machine_context = threading.RLock()

		self.machine = Machine(model=self, states=DriverLogic.states, transitions=DriverLogic.transitions,
							   initial='not_initialized', machine_context=[self.state_machine_context])
		self.serial_port_comm = serial_port_comm
		serial_port_comm.add_flags_observer(self)
		self.gripper_pos = None

		self.opening_pos = 29.5 # default gripper values
		self.closing_pos = 0.5

		self.service_sync = threading.Lock()
		self.operation_params = None
		self.operation_response = None
		self.async_operation_finished = threading.Condition()
		self.no_spurious_wakeup = False

		self.old_flag_signaled = {"OPEN_SIGNALED":False, "CLOSED_SIGNALED":False, "HOLDING_SIGNALED":False,
								  "FAULT_SIGNALED":False, "IDLE_SIGNALED":False, "OTHER_FAULT_SIGNALED":False,
								  "NOT_INITIALIZED_SIGNALED":True}

	def service_called(self, transition, params, trigger_response):
		with self.service_sync: # only one service call allowed at a time
			self.no_spurious_wakeup = False
			self.operation_params = params
			self.operation_response = trigger_response
			with self.async_operation_finished: # needed if result of service only known at later stage
				try:
					trigger_response.success = False # only successful if explicitly set
					if self.trigger(transition):
						while not self.no_spurious_wakeup: # make sure, the wait finishes correctly
							self.async_operation_finished.wait() # needed if result of service known at later stage
				except transitions.core.MachineError as err: # not a valid action in the current state
					trigger_response.success = False
					trigger_response.message = self.get_err_msg(self.state) + trigger_response.message
				finally:
					if trigger_response.success:
						trigger_response.message = "succeeded. " + trigger_response.message
					else:
						trigger_response.message = "failed. " + trigger_response.message
					self.operation_params = None
					self.operation_response = None

	def operation_finished(self):
		with self.async_operation_finished:
			self.no_spurious_wakeup = True
			self.async_operation_finished.notify()

	def operation_successful(self):
		self.operation_response.success = True

	def claws_blocked(self):
		self.operation_response.success = False
		self.operation_response.message += 'Claws are blocked.'

	def no_object(self):
		self.operation_response.success = False
		self.operation_response.message += 'No object to grasp.'

	def unexpected_state_change(self):
		if self.operation_response is not None:
			self.operation_response.success = False
			self.operation_response.message += 'Unexpected state change.'
		rospy.logerr('Unexpected state change.')

	def get_err_msg(self, state):
		operation_err_msg_from_state = {'not_initialized': 'Reference the gripper before usage. ',
										'fault': 'Fault state. Ack the error before proceeding. ',
										'other_fault': 'Temperature error or maintenance required. No operation possible. '}

		if state in operation_err_msg_from_state:
			return operation_err_msg_from_state[state]
		else:
			return "Operation not allowed from current state ({}). ".format(state)

	def update_flags(self, new_flags_dict):
			thr = threading.Thread(target=self.update_flags_thread, args=(new_flags_dict,))
			thr.start()

	def update_flags_thread(self, new_flags_dict):
		self.gripper_pos = new_flags_dict["POS"]

		# the observer function should be non-blocking
		# -> make sure the state machine is not blocked by a service call
		if not self.state_machine_context.acquire(blocking=False):
			return

		if new_flags_dict["TEMPFAULT_FLAG"] == 1 or new_flags_dict["MAINT_FLAG"]: #or new_flags_dict["TEMPWARN_FLAG"]:
			if not self.old_flag_signaled["OTHER_FAULT_SIGNALED"]:
				self.set_signaled_flag("OTHER_FAULT_SIGNALED")
				self.to_other_fault()
		elif new_flags_dict["FAULT_FLAG"] == 1:
			if not self.old_flag_signaled["FAULT_SIGNALED"]:
				self.set_signaled_flag("FAULT_SIGNALED")
				self.to_fault()
		elif new_flags_dict["IDLE_FLAG"] == 1:
			if not self.old_flag_signaled["IDLE_SIGNALED"]:
				self.set_signaled_flag("IDLE_SIGNALED")
				self.on_inactive()
		elif new_flags_dict["OPEN_FLAG"] == 1:
			if not self.old_flag_signaled["OPEN_SIGNALED"]:
				self.set_signaled_flag("OPEN_SIGNALED")
				self.on_open()
		elif new_flags_dict["CLOSED_FLAG"] == 1:
			if not self.old_flag_signaled["CLOSED_SIGNALED"]:
				self.set_signaled_flag("CLOSED_SIGNALED")
				self.on_closed()
		elif new_flags_dict["HOLDING_FLAG"] == 1:
			if not self.old_flag_signaled["HOLDING_SIGNALED"]:
				self.set_signaled_flag("HOLDING_SIGNALED")
				self.on_holding()
		elif not self.check_if_referenced(new_flags_dict):
			if not self.old_flag_signaled["NOT_INITIALIZED_SIGNALED"]:
				self.set_signaled_flag("NOT_INITIALIZED_SIGNALED")
				self.on_not_initialized()

		self.state_machine_context.release()

	def set_signaled_flag(self, flag):
		rospy.loginfo("signaled flag: {}".format(flag))
		self.old_flag_signaled["OTHER_FAULT_SIGNALED"] = False
		self.old_flag_signaled["FAULT_SIGNALED"] = False
		self.old_flag_signaled["IDLE_SIGNALED"] = False
		self.old_flag_signaled["OPEN_SIGNALED"] = False
		self.old_flag_signaled["CLOSED_SIGNALED"] = False
		self.old_flag_signaled["HOLDING_SIGNALED"] = False
		self.old_flag_signaled["NOT_INITIALIZED_SIGNALED"] = False
		self.old_flag_signaled[flag] = True

	def check_if_referenced(self, flags_dict):
		#when the gripper is not referenced all flags are 0
		gripper_referenced = False

		if (flags_dict["IDLE_FLAG"] == 1 or flags_dict["OPEN_FLAG"] == 1 or
				flags_dict["CLOSED_FLAG"] == 1 or flags_dict["HOLDING_FLAG"] == 1 or
				flags_dict["FAULT_FLAG"] == 1 or flags_dict["TEMPFAULT_FLAG"] == 1 or
				flags_dict["TEMPWARN_FLAG"] == 1 or flags_dict["MAINT_FLAG"] == 1):
			gripper_referenced = True

		return gripper_referenced

	def can_move_while_holding(self):
		outer_grip = self.opening_pos > self.closing_pos
		if outer_grip and self.operation_params.position > self.gripper_pos:
			return True # if currently outer grip -> only allow movement to outer side
		elif not outer_grip and self.operation_params.position < self.gripper_pos:
			return True # if currently inner grip -> only allow movement to inner side
		else:
			self.operation_response.success = False
			self.operation_response.message += 'Moving into the grasped object is not possible. Move in the other direction. Current pos: {}, commanded pos: {}'.format(self.gripper_pos, self.operation_params.position)
			return False

	def can_grasp(self):
		least_movement = 0.5 # margin found by trial-and-error
		if np.abs(self.operation_params.position - self.gripper_pos) >= least_movement:
			return True
		else:
			self.operation_response.success = False
			self.operation_response.message += 'Grasping range too little. Move by at least {}mm.. Current pos: {}, commanded pos: {}'.format(least_movement, self.gripper_pos, self.operation_params.position)
			return False

	def exec_activating(self):
		rospy.loginfo('State: {}'.format(self.state))
		self.serial_port_comm.send_command_synced("activate")

	def exec_deactivating(self):
		rospy.loginfo('State: {}'.format(self.state))
		self.serial_port_comm.send_command_synced("deactivate")

	def exec_referencing(self):
		rospy.loginfo('State: {}'.format(self.state))
		self.serial_port_comm.send_command_synced("reference")

	def set_positions(self, opening_pos, closing_pos):
		pos_set = False
		while not pos_set:
			pos_set = self.serial_port_comm.set_opening_pos(opening_pos)
			pos_set = pos_set and self.serial_port_comm.set_closing_pos(closing_pos)
		self.opening_pos = opening_pos
		self.closing_pos = closing_pos
		rospy.loginfo("Opening pos: {}, closing pos: {}".format(opening_pos, closing_pos))

	def exec_opening(self):
		rospy.loginfo('Trying to set opening positions...')
		rospy.loginfo('State: {}'.format(self.state))
		self.set_positions(opening_pos=self.operation_params.position, closing_pos=self.gripper_pos)
		rospy.loginfo('Opening positions set.')
		self.serial_port_comm.send_command_synced("open")

	def exec_closing(self):
		rospy.loginfo('Trying to set closing positions...')
		rospy.loginfo('State: {}'.format(self.state))
		self.set_positions(opening_pos=self.gripper_pos, closing_pos=self.operation_params.position)
		rospy.loginfo('Closing positions set.')
		self.serial_port_comm.send_command_synced("close")

	def exec_grasping(self):
		rospy.loginfo('Trying to set grasping positions...')
		rospy.loginfo('State: {}'.format(self.state))
		self.set_positions(opening_pos=self.gripper_pos, closing_pos=self.operation_params.position)
		rospy.loginfo('Grasping positions set.')
		self.serial_port_comm.send_command_synced("close")

	def exec_opening_before_closing(self):
		rospy.loginfo('Trying to set opening_before_closing positions...')
		rospy.loginfo('State: {}'.format(self.state))
		self.set_positions(opening_pos=self.gripper_pos, closing_pos=self.gripper_pos)
		rospy.loginfo('Prepening positions set.')
		self.serial_port_comm.send_command_synced("open")

	def exec_closing_before_opening(self):
		rospy.loginfo('Trying to set closing_before_opening positions...')
		rospy.loginfo('State: {}'.format(self.state))
		self.set_positions(opening_pos=self.gripper_pos, closing_pos=self.gripper_pos)
		rospy.loginfo('Closing_before_opening positions set.')
		self.serial_port_comm.send_command_synced("close")
