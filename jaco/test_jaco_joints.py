import control_msgs
import rospy

s = rospy.advertise("/jaco/jaco_arm_controller/joint_trajectory_action/goal", control_msgs.msg.FollowJointTrajectoryActionGoal);
s.publish(header=[stamp: now], goal=[trajectory=[header=[seq=1, stamp=now, frame_id=''], joint_names=['jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6', 'jaco_finger_joint_1', 'jaco_finger_joint_2', 'jaco_finger_joint_3'], points= [positions= [2.05, -1.94, -0.79, 0.85, -1.67, -3.13, 0.00, 0.00, 0.00], velocities: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0] ] ]]]
