class RobotScoringPositions:
	elevator_raise_threshold = 1.0 # meters needed to be closer than to reef to raise elevator
	end_effector_travel_position = 0.0 # 8.0
	elevator_intake_height = 2.0
	end_effector_intake_position = 0.0
	min_elevator_height_to_bring_in_end_effector = 2.2
	min_end_effector_position_to_move_elevator_up = 8.2
	class L1_Scoring:
		elevator_height = 7.18
		end_effector_outtake_speed = 20
		end_effector_position = 10.0
		number = 1
	class L2_Scoring:
		elevator_height = 23.0
		end_effector_outtake_speed = 35
		end_effector_position = 10.0
		number = 2
	class L3_Scoring:
		elevator_height = 39.5
		end_effector_outtake_speed = 35
		end_effector_position = 10.0
		number = 3
	class L4_Scoring:
		elevator_height = 61.0
		end_effector_outtake_speed = 42.0
		end_effector_position = 10.0
		number = 4