# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot
# ------------------------------------------------

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
)

import math
from wpilib import Timer

from wpilibextra.coroutine import commandify
import oi


class Coroutines:
    """
    Coroutines - This class defines coroutines commands that are used in robot code.
    """

    def __init__(self, robot: "Robot"):
        self.auto_drive_lock = False
        self.auto_note_lock = False


        # Example for when we begin working on autonomous mode
        # @commandify(requirements=[robot.ground_intake, robot.over_bumper_intake])
        # def ground_intake_back_passthrough_command():
        #     yield
        #     robot.ground_intake.back_passthrough()

        # self.ground_intake_back_passthrough_command = (
        #     ground_intake_back_passthrough_command()
        # )