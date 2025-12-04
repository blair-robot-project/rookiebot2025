package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.auto.Routines;
import frc.robot.subsystems.SwerveDriveSubsytem;

class Auto (
    /*val robot: Robot,
    val drive: SwerveDriveSubsytem,*/
) {
    //val routines: Routines = Routines(robot, null)

    // Called when auto starts
    fun autoInit() {
    }

    // Called when auto ends
    fun autoEnd() {
    }
}