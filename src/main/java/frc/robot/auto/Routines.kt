package frc.robot.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveDriveSubsytem

class PID

class Routines (
    val robot: Robot,
    val drive: SwerveDriveSubsytem,
) {
    private val autoFactory = AutoFactory(
        null,
        null,
        null,
        false,
        drive,
    )

    /*public AutoRoutine aaaaaaa() {
        routine: AutoRoutine = autoFactory.newRoutine ("aaaaa")
    }*/
}