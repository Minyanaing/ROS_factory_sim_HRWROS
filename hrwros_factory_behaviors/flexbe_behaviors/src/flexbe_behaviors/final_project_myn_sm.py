#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit as flexbe_manipulation_states__SrdfStateToMoveit
from hrwros_factory_states.control_feeder_state import ControlFeederState
from hrwros_factory_states.vacuum_gripper_control_state import VacuumGripperControlState
from hrwros_factory_states.compute_grasp_state import ComputeGraspState
from hrwros_factory_states.locate_factory_device_state import LocateFactoryDeviceState
from hrwros_factory_states.detect_part_camera_state import DetectPartCameraState
from flexbe_states.subscriber_state import SubscriberState
from hrwros_factory_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as hrwros_factory_states__MoveitToJointsDynState
from hrwros_factory_states.set_conveyor_power_state import SetConveyorPowerState
from hrwros_factory_states.move_base_state import MoveBaseState as hrwros_factory_states__MoveBaseState
from hrwros_factory_states.srdf_state_to_moveit import SrdfStateToMoveit as hrwros_factory_states__SrdfStateToMoveit
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D

# [/MANUAL_IMPORT]


'''
Created on 19/07/2020
@author: Min Yan Naing
'''
class FinalProject_MYNSM(Behavior):
	'''
	Complete final project for the MOOC Hello (Real) World with ROS.
	'''


	def __init__(self):
		super(FinalProject_MYNSM, self).__init__()
		self.name = 'Final Project_MYN'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		names1 = ['robot1_shoulder_pan_joint', 'robot1_shoulder_lift_joint', 'robot1_elbow_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']
		pick1_group = 'robot1'
		robot1_loc = Pose2D(x=3.8, y=2.1, theta=-90.0)
		gripper1 = "vacuum_gripper1_suction_cup"
		pick2_group = 'robot2'
		robot1_loc2 = Pose2D(x=-4.3, y=-0.9, theta=0.0)
		names2 = ['robot2_shoulder_pan_joint', 'robot2_shoulder_lift_joint', 'robot2_elbow_joint', 'robot2_wrist_1_joint', 'robot2_wrist_2_joint', 'robot2_wrist_3_joint']
		gripper2 = "vacuum_gripper2_suction_cup"
		# x:265 y:202, x:599 y:137
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.robot1_loc = robot1_loc
		_state_machine.userdata.pose_turtlebot = []
		_state_machine.userdata.pick1_configuration = []
		_state_machine.userdata.place1_configuration = []
		_state_machine.userdata.speed_conveyor = 100
		_state_machine.userdata.robot1_loc2 = robot1_loc2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:50 y:30
			OperatableStateMachine.add('Move R1 Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='R1Home', move_group=pick1_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'Deactivate_R2_Bin_2', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:442 y:20
			OperatableStateMachine.add('Start feeder',
										ControlFeederState(activation=True),
										transitions={'succeeded': 'Wait for part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:804 y:18
			OperatableStateMachine.add('Stop feeder',
										ControlFeederState(activation=False),
										transitions={'succeeded': 'Stop_conveyor', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1161 y:170
			OperatableStateMachine.add('Activate Gripper 1',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'Move R1 to pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1169 y:463
			OperatableStateMachine.add('Compute place Turtlebot',
										ComputeGraspState(group=pick1_group, offset=0.6, joint_names=names1, tool_link=gripper1, rotation=3.1415),
										transitions={'continue': 'Move R1 to place', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose_turtlebot', 'joint_values': 'place1_configuration', 'joint_names': 'joint_names'})

			# x:1166 y:370
			OperatableStateMachine.add('LocateTurtlebot',
										LocateFactoryDeviceState(model_name='mobile_base', output_frame_id='world'),
										transitions={'succeeded': 'Compute place Turtlebot', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose_turtlebot'})

			# x:1163 y:276
			OperatableStateMachine.add('Move R1 back Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='R1Home', move_group=pick1_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'Navigate_to_Robot1', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:489 y:306
			OperatableStateMachine.add('Move R2 back to Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='R2Home', move_group=pick2_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'Move_R2_to_Bin', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1177 y:47
			OperatableStateMachine.add('Detect Part Camera',
										DetectPartCameraState(ref_frame='robot1_base', camera_topic='/hrwros/logical_camera_1', camera_frame='logical_camera_1_frame'),
										transitions={'continue': 'Compute pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:643 y:22
			OperatableStateMachine.add('Wait for part',
										SubscriberState(topic='/break_beam_sensor_change', blocking=True, clear=True),
										transitions={'received': 'Stop feeder', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:997 y:238
			OperatableStateMachine.add('Move R1 to pick',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick1_group, offset=0.0, tool_link=gripper1, action_topic='/move_group'),
										transitions={'reached': 'Move R1 back Home', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick1_configuration', 'joint_names': 'joint_names'})

			# x:977 y:439
			OperatableStateMachine.add('Move R1 to place',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick1_group, offset=0.0, tool_link=gripper1, action_topic='/move_group'),
										transitions={'reached': 'Deactivate Gripper 1', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'place1_configuration', 'joint_names': 'joint_names'})

			# x:258 y:18
			OperatableStateMachine.add('Start_conveyor',
										SetConveyorPowerState(stop=False),
										transitions={'succeeded': 'Start feeder', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'speed_conveyor'})

			# x:983 y:23
			OperatableStateMachine.add('Stop_conveyor',
										SetConveyorPowerState(stop=True),
										transitions={'succeeded': 'Detect Part Camera', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'speed_conveyor'})

			# x:996 y:339
			OperatableStateMachine.add('Navigate_to_Robot1',
										hrwros_factory_states__MoveBaseState(),
										transitions={'arrived': 'LocateTurtlebot', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'robot1_loc'})

			# x:995 y:125
			OperatableStateMachine.add('Compute pick',
										ComputeGraspState(group=pick1_group, offset=0.0, joint_names=names1, tool_link=gripper1, rotation=3.1415),
										transitions={'continue': 'Activate Gripper 1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick1_configuration', 'joint_names': 'joint_names'})

			# x:817 y:330
			OperatableStateMachine.add('Deactivate Gripper 1',
										VacuumGripperControlState(enable=False, service_name='/gripper1/control'),
										transitions={'continue': 'Move R1 back Home_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:828 y:138
			OperatableStateMachine.add('Move R1 back Home_2',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='R1Home', move_group=pick1_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'Navigate_to_Loc2', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:721 y:235
			OperatableStateMachine.add('Navigate_to_Loc2',
										hrwros_factory_states__MoveBaseState(),
										transitions={'arrived': 'Detect_box_Cam2', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'robot1_loc2'})

			# x:661 y:379
			OperatableStateMachine.add('Detect_box_Cam2',
										DetectPartCameraState(ref_frame='robot2_base', camera_topic='/hrwros/logical_camera_2', camera_frame='logical_camera_2_frame'),
										transitions={'continue': 'Compute_pick_Arm2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:793 y:486
			OperatableStateMachine.add('Compute_pick_Arm2',
										ComputeGraspState(group=pick2_group, offset=0.0, joint_names=names2, tool_link=gripper2, rotation=3.1415),
										transitions={'continue': 'Activate_Grip2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick1_configuration', 'joint_names': 'joint_names'})

			# x:568 y:484
			OperatableStateMachine.add('Activate_Grip2',
										VacuumGripperControlState(enable=True, service_name='/gripper2/control'),
										transitions={'continue': 'Move_Robot2_Pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:510 y:403
			OperatableStateMachine.add('Move_Robot2_Pick',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick2_group, offset=0.0, tool_link=gripper2, action_topic='/move_group'),
										transitions={'reached': 'Move R2 back to Home', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick1_configuration', 'joint_names': 'joint_names'})

			# x:142 y:117
			OperatableStateMachine.add('Deactivate_Grip1_init',
										VacuumGripperControlState(enable=False, service_name='/gripper1/control'),
										transitions={'continue': 'Start_conveyor', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:367 y:373
			OperatableStateMachine.add('Move_R2_to_Bin',
										hrwros_factory_states__SrdfStateToMoveit(config_name='R2Place', move_group=pick2_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'Deactivate_R2_Bin', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:336 y:468
			OperatableStateMachine.add('Deactivate_R2_Bin',
										VacuumGripperControlState(enable=False, service_name='/gripper2/control'),
										transitions={'continue': 'Move R2 back to Home_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:160 y:460
			OperatableStateMachine.add('Move R2 back to Home_2',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='R2Home', move_group=pick2_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:30 y:181
			OperatableStateMachine.add('Deactivate_R2_Bin_2',
										VacuumGripperControlState(enable=False, service_name='/gripper2/control'),
										transitions={'continue': 'Deactivate_Grip1_init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
