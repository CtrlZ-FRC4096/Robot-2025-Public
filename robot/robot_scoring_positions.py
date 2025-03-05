class RobotScoringPositions:
	elevator_raise_threshold = 1.0 # meters needed to be closer than to reef to raise elevator
	end_effector_travel_position = 8.1 # 8.0
	elevator_intake_height = 3.0
	end_effector_intake_position = 0.0
	min_elevator_height_to_bring_in_end_effector = 4.0
	min_end_effector_position_to_move_elevator_up = 7.5
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
		elevator_height = 61.5
		end_effector_outtake_speed = 42.0
		end_effector_position = 10.0
		number = 4