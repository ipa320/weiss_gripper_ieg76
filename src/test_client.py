#!/usr/bin/env python
import roslib;
import sys
import rospy
import math
from std_srvs.srv import Trigger
from weiss_gripper_ieg76.srv import *

def send_reference_request():
	rospy.wait_for_service('reference')
	try:
			reference = rospy.ServiceProxy('reference', Trigger)
			resp = reference()
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def send_close_request(grasp_config_no):
	rospy.wait_for_service('close_jaws')
	try:
			close_jaws = rospy.ServiceProxy('close_jaws', ConfigTrigger)
			resp = close_jaws(grasp_config_no)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def send_grasp_object_request(grasp_config_no):
	rospy.wait_for_service('grasp_object')
	try:
			grasp_object = rospy.ServiceProxy('grasp_object', ConfigTrigger)
			resp = grasp_object(grasp_config_no)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def send_open_request(grasp_config_no):
	rospy.wait_for_service('open_jaws')
	try:
			open_jaws = rospy.ServiceProxy('open_jaws', ConfigTrigger)
			resp = open_jaws(grasp_config_no)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def send_ack_error_request():
	rospy.wait_for_service('ack_error')
	try:
			ack_error = rospy.ServiceProxy('ack_error', Trigger)
			resp = ack_error()
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def send_ack_ref_error_request():
	rospy.wait_for_service('ack_ref_error')
	try:
			ack_ref_error = rospy.ServiceProxy('ack_ref_error', Trigger)
			resp = ack_ref_error()
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def get_all_param_request(grasp_config_no):
	rospy.wait_for_service('get_all_param')
	try:
			get_all_param = rospy.ServiceProxy('get_all_param', GetAllParam)
			resp = get_all_param(grasp_config_no)
			if resp.success == True:
				print("Success: " + resp.message)
				print("grasping_force: " + str(resp.grasping_force) + " [%]")
				print("opening_position: " + str(round(resp.opening_position, 2)) + " [mm]")
				print("closing_position: " + str(round(resp.closing_position, 2)) + " [mm]")
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def set_all_param_request(grasp_config_no, grasping_force, opening_position, closing_position):
	rospy.wait_for_service('set_all_param')
	try:
			set_all_param = rospy.ServiceProxy('set_all_param', SetAllParam)
			resp = set_all_param(grasp_config_no, grasping_force, opening_position, closing_position)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def set_grasping_force_request(grasp_config_no, grasping_force):
	rospy.wait_for_service('set_grasping_force')
	try:
			set_grasping_force = rospy.ServiceProxy('set_grasping_force', SetGraspingForce)
			resp = set_grasping_force(grasp_config_no, grasping_force)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def set_opening_pos_request(grasp_config_no, opening_position):
	rospy.wait_for_service('set_opening_pos')
	try:
			set_opening_pos = rospy.ServiceProxy('set_opening_pos', SetOpeningPos)
			resp = set_opening_pos(grasp_config_no, opening_position)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def set_closing_pos_request(grasp_config_no, closing_position):
	rospy.wait_for_service('set_closing_pos')
	try:
			set_closing_pos = rospy.ServiceProxy('set_closing_pos', SetClosingPos)
			resp = set_closing_pos(grasp_config_no, closing_position)
			if resp.success == True:
				print("Success: " + resp.message)
			else:
				print("Failure: " + resp.message)
	except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

def usage():
	print("-------- Commands Weiss Robotics Gripper ieg76 --------")
	print("1. Reference")
	print("2. Open jaws")
	print("3. Close jaws")
	print("4. Grasp object")
	print("5. Ack error")
	print("6. Ack reference error")
	print("7. Select grasp configuration")
	print("8. Get all the grasp configuration's param")
	print("9. Set all the grasp configuration's param")
	print("10. Set the grasping force")
	print("11. Set the opening position")
	print("12. Set the closing position")
	print("13. Exit")

if __name__ == "__main__":
	selected_cmd = usage()
	grasp_config_no = 0
	while True:
		selected_cmd = eval(input("Select a command to send: "))
		if selected_cmd == 1:
			print("Sending reference request...")
			send_reference_request()
		elif selected_cmd == 2:
			print("Sending open jaws request...")
			send_open_request(grasp_config_no)
		elif selected_cmd == 3: 
			print("Sending close jaws request...")
			send_close_request(grasp_config_no)
		elif selected_cmd == 4: 
			print("Sending grasp object request...")
			send_grasp_object_request(grasp_config_no)
		elif selected_cmd == 5: 
			print("Sending acknowledge error request...")
			send_ack_error_request()
		elif selected_cmd == 6: 
			print("Sending acknowledge error request...")
			send_ack_ref_error_request()
		elif selected_cmd == 7: 
			print("Current grasp configuration number: " + str(grasp_config_no))
			input_data = eval(input("Enter new grasp configuration no = "))
			if input_data >=0 and input_data <=3:
				grasp_config_no = input_data
			else:
				print("Unknown grasp config no " + str(input_data) + " given. Valid grasp configuration numbers are 0, 1, 2, 3.")
		elif selected_cmd == 8: 
			print("Getting all the parameters (grasping force, opening position, closing position) of grasp configuration number " + str(grasp_config_no) + "...")
			get_all_param_request(grasp_config_no)
		elif selected_cmd == 9: 
			print("Setting all the parameters (grasping force, opening position, closing position) of grasp configuration number " + str(grasp_config_no) + "...")
			input_data_grasping_force = eval(input("Enter the grasping force (0..100%) of grasp config no " + str(grasp_config_no) + " = "))
			input_data_opening_position = eval(input("Enter the opening position of grasp config no " + str(grasp_config_no) + " = "))
			input_data_closing_position = eval(input("Enter the closing position of grasp config no " + str(grasp_config_no) + " = "))
			if input_data_grasping_force <0 or input_data_grasping_force >100:
				print("Value error " + str(input_data_grasping_force) + ". Valid grasping force values are 0..100%.")
			elif input_data_opening_position <0.0 and input_data_opening_position >30.0:
				print("Value error " + str(input_data_opening_position) + ". Valid opening position values are 0.0..30.0 mm.")
			elif input_data_closing_position <0.0 and input_data_closing_position >30.0:
				print("Value error " + str(input_data_closing_position) + ". Valid closing position values are 0.0..30.0 mm.")
			else:
				print("Setting all the param (grasping force " + str(input_data_grasping_force) + " [%], opening position " + str(input_data_opening_position) + " [mm], closing position " + str(input_data_closing_position) + " [mm]) of grasp config " + str(grasp_config_no) + ".")
				set_all_param_request(grasp_config_no, input_data_grasping_force, input_data_opening_position, input_data_closing_position)
		elif selected_cmd == 10:
			input_data = eval(input("Enter the grasping force (0..100%) of grasp config no " + str(grasp_config_no) + " = "))
			if input_data >=0 and input_data <=100:
				print("Setting the grasping force of grasp configuration number " + str(grasp_config_no) + "...")
				set_grasping_force_request(grasp_config_no, input_data)
			else:
				print("Value error " + str(input_data) + ". Valid grasping force values are 0..100%.") 
		elif selected_cmd == 11:
			input_data = eval(input("Enter the opening position of grasp config no " + str(grasp_config_no) + " = "))
			if input_data >=0.0 and input_data <=30.0:
				print("Setting the opening position of grasp configuration number " + str(grasp_config_no) + "...")
				set_opening_pos_request(grasp_config_no, input_data)
			else:
				print("Value error " + str(input_data) + ". Valid opening position values are 0.0..30.0 mm.")
		elif selected_cmd == 12: 
			input_data = eval(input("Enter the closing position of grasp config no " + str(grasp_config_no) + " = "))
			if input_data >=0.0 and input_data <=30.0:
				print("Setting the closing position of grasp configuration number " + str(grasp_config_no) + "...")
				set_closing_pos_request(grasp_config_no, input_data)
			else:
				print("Value error " + str(input_data) + ". Valid closing position values are 0.0..30.0 mm.")
		elif selected_cmd == 13:
			print("Exiting the test_client...")
			sys.exit(0)
		else:
			print("Unknown option entered.")