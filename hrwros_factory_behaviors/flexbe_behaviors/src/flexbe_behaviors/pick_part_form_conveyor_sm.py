#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hrwros_factory_states.detect_part_camera_state import DetectPartCameraState
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as flexbe_manipulation_states__MoveitToJointsDynState
from hrwros_factory_states.vacuum_gripper_control_state import VacuumGripperControlState
from hrwros_factory_states.compute_grasp_state import ComputeGraspState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 10 2020
@author: Min Yan Naing
'''
class PickpartformconveyorSM(Behavior):
	'''
	Robot Arm 1 flexBE setup.
	'''


	def __init__(self):
		super(PickpartformconveyorSM, self).__init__()
		self.name = 'Pick part form conveyor'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'robot1'
		home1 = [1.57, -1.57, 1.24, -1.57, -1.57, 0]
		names1 = ['robot1_shoulder_pan_joint', 'robot1_shoulder_lift_joint', 'robot1_elbow_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']
		gripper1 = "vacuum_gripper1_suction_cup"
		# x:1080 y:384, x:225 y:414
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = []
		_state_machine.userdata.home1 = home1
		_state_machine.userdata.conveyor_speed = 100

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:189 y:44
			OperatableStateMachine.add('Detect Part',
										DetectPartCameraState(ref_frame='robot1_base', camera_topic='/hrwros/logical_camera_1', camera_frame='logical_camera_1_frame'),
										transitions={'continue': 'ComputePoseConf:', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:623 y:170
			OperatableStateMachine.add('Move Robot1 to pick',
										flexbe_manipulation_states__MoveitToJointsDynState(move_group=pick_group, action_topic='/move_group'),
										transitions={'reached': 'ActivateGripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:659 y:263
			OperatableStateMachine.add('ActivateGripper',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'Move Robot to home configuration', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:628 y:362
			OperatableStateMachine.add('Move Robot to home configuration',
										flexbe_manipulation_states__MoveitToJointsDynState(move_group=pick_group, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'home1', 'joint_names': 'joint_names'})

			# x:598 y:77
			OperatableStateMachine.add('ComputePoseConf:',
										ComputeGraspState(group=pick_group, offset=0.0, joint_names=names1, tool_link=gripper1, rotation=3.1415),
										transitions={'continue': 'Move Robot1 to pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
