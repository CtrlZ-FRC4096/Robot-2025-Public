class RobotScoringPositions:
	elevator_raise_threshold = 2 # meters needed to be closer than to reef to raise elevator
	end_effector_travel_position = 0.0
	elevator_intake_height = 0.14
	class L1_Scoring:
		elevator_height = 1
		end_effector_outtake_speed = 15
		end_effector_position = 4
		number = 1
	class L2_Scoring:
		elevator_height = 3
		end_effector_outtake_speed = 10
		end_effector_position = 4
		number = 2
	class L3_Scoring:
		elevator_height = 6
		end_effector_outtake_speed = 10
		end_effector_position = 4
		number = 3
	class L4_Scoring:
		elevator_height = 10
		end_effector_outtake_speed = 15
		end_effector_position = 4
		number = 4