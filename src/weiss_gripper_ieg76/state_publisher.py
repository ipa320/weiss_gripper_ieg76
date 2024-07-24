import rospy
import threading
import diagnostic_updater
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus


class StatesPublisher(threading.Thread):
    def __init__(self, loop_time, serial_port_comm):
        threading.Thread.__init__(self)
        self.loop_time = loop_time
        self.driver_shutdown = False

        self.current_flags_sync = threading.Lock()
        self.current_flags_dict = None
        self.current_flags_updated = False
        serial_port_comm.add_flags_observer(self)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = []
        self.joint_state_msg.name.append("ur5_weiss_ieg76_jaw_position")  # TODO use parameter
        self.joint_states_publisher = rospy.Publisher('~joint_states', JointState, queue_size=10)

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Weiss Robotics Gripper ??? SN ???")
        self.updater.add("Position and flags updater", self.produce_diagnostics)
        freq_bounds = {'min': 0.5, 'max': 2}
        # It publishes the messages and simultaneously makes diagnostics for the topic "joint_states" using a FrequencyStatus and TimeStampStatus
        self.pub_freq_time_diag = diagnostic_updater.DiagnosedPublisher(self.joint_states_publisher,
                                                        self.updater,
                                                        diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10),
                                                        diagnostic_updater.TimeStampStatusParam())

    def set_gripper_info(self, gripper_info):
        self.updater.setHardwareID("Weiss Robotics Gripper {} SN {}".format(gripper_info["product_id"], gripper_info["serial_no"]))

    def shutdown(self):
        self.driver_shutdown = True

    def update_flags(self, new_flags_dict):
        with self.current_flags_sync:
            self.current_flags_dict = new_flags_dict
            self.current_flags_updated = True

    def run(self):
        rospy.logdebug("StatesPublisher.run()")
        while not self.driver_shutdown:
            with self.current_flags_sync:
                if self.current_flags_updated:
                    self.updater.update()
                    rospy.logdebug("StatesPublisher publishing states...")
                    self.publish_states()
                    rospy.logdebug("StatesPublisher states published.")
                    #self.updater.force_update()
                    self.current_flags_updated = False
            rospy.sleep(self.loop_time)
        rospy.logdebug("states_publisher_thread done.")

    def produce_diagnostics(self, stat):
        if self.current_flags_dict["FAULT_FLAG"] == True:
            stat.summary(DiagnosticStatus.ERROR, "The fault bit of the gripper is 1.")
        else:
            stat.summary(DiagnosticStatus.OK, "The fault bit of the gripper is 0.")
        stat.add("Position", self.current_flags_dict["POS"])
        stat.add("Idle Flag", self.current_flags_dict["IDLE_FLAG"])
        stat.add("Open Flag", self.current_flags_dict["OPEN_FLAG"])
        stat.add("Closed Flag", self.current_flags_dict["CLOSED_FLAG"])
        stat.add("Holding Flag", self.current_flags_dict["HOLDING_FLAG"])
        stat.add("Error Flag", self.current_flags_dict["FAULT_FLAG"])
        stat.add("Temperature Error Flag", self.current_flags_dict["TEMPFAULT_FLAG"])
        stat.add("Temperature Warning Flag",
                 self.current_flags_dict["TEMPWARN_FLAG"])
        stat.add("Maintenance Flag", self.current_flags_dict["MAINT_FLAG"])
        return stat

    def publish_states(self):
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.position = []
        MM_TO_M = 0.001
        joint_state = self.current_flags_dict["POS"] * MM_TO_M / 2.0  # joint state is half the stroke ("POS")
        self.joint_state_msg.position.append(joint_state)
        try:
            self.pub_freq_time_diag.publish(self.joint_state_msg)
        except:
            rospy.logerr("\nClosed topics.")
