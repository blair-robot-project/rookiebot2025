package frc.robot.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveDriveSubsytem
import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.constants.SwerveDriveConstants
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.kinematics.ChassisSpeeds


class Routines (
    val robot: Robot,
    val drive: SwerveDriveSubsytem,
) {

    // Psuedo PID Controller
    val xController: PIDController = PIDController(SwerveDriveConstants.kp, SwerveDriveConstants.ki,
        SwerveDriveConstants.kd)
    val yController: PIDController = PIDController(SwerveDriveConstants.kp, SwerveDriveConstants.ki,
        SwerveDriveConstants.kd)
    val headingController: PIDController = PIDController(SwerveDriveConstants.kp, SwerveDriveConstants.ki,
        SwerveDriveConstants.kd)

    fun followTrajectory(sample: SwerveSample) {
        // Get pose from swerve

        val speeds: ChassisSpeeds = ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        )

        val newSpeeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            // Robot angle
        )

        // Apply gen speed
        // Get set fun from swerve set(newSpeeds)
    }


    private val autoFactory = AutoFactory(
        null,
        null,
        { sample: SwerveSample -> followTrajectory(/*Get Sample*/)},
        false,
        drive,
    )

    // Routine for position one blue team
     fun positionOneBlue(): AutoRoutine {
         val routine: AutoRoutine = autoFactory.newRoutine("positionOneBlue")

         routine.active().onTrue(
             Commands.sequence(

             )
         )

         return routine
    }
}