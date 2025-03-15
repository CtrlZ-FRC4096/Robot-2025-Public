class RobotScoringPositions:
	elevator_raise_threshold = 1.6 # meters needed to be closer than to reef to raise elevator
	end_effector_travel_position = 8.1 # 8.0
	elevator_intake_height = 2.0
	end_effector_intake_position = 0.0
	min_elevator_height_to_bring_in_end_effector = 4.0
	min_end_effector_position_to_move_elevator_up = 7.5
	dangerous_tip_angle = 15.0
	end_effector_climbing_position = 10.0
	elevator_climb_height = 13.0
	min_end_effector_position_to_climb = 7.7
	min_elevator_climb_height = 10.0
	climber_up_position = 0.5577
	climber_climb_position = 0.2072
	climber_default_position = 0.0
	class L1_Scoring:
		elevator_height = 15.0
		end_effector_outtake_speed = 15
		end_effector_position = 10.0
		number = 1
	class L2_Scoring:
		elevator_height = 23.0
		end_effector_outtake_speed = 35
		end_effector_position = 10.0
		number = 2
	class L3_Scoring:
		elevator_height = 39.0
		end_effector_outtake_speed = 35
		end_effector_position = 10.0
		number = 3
	class L4_Scoring:
		elevator_height = 61.0
		end_effector_outtake_speed = 42.0
		end_effector_position = 10.0
		number = 4
	class Descore_Algae_L3:
		elevator_height = 37.5
		end_effector_outtake_speed = 40
		end_effector_position = 8.5
		number = 6
	class Descore_Algae_L2:
		elevator_height = 20.0
		end_effector_outtake_speed = 40
		end_effector_position = 8.5
		number = 5