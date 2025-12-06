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
        val pOneBlueTrajectory = routine.trajectory("1s(b)")
         routine.active().onTrue(
             Commands.sequence(
                 //shooterClass.functionThatShoots
                 pOneBlueTrajectory.resetOdometry(),
                 pOneBlueTrajectory.cmd()
                 //swerveClass.functionThatStops
             )
         )
         return routine
    }

    fun positionTwoBlue(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("positionTwoBlue")
        val pTwoBlueTrajectory = routine.trajectory("2s(b)")
        routine.active().onTrue(
            Commands.sequence(
                //shooterClass.functionThatShoots
                pTwoBlueTrajectory.resetOdometry(),
                pTwoBlueTrajectory.cmd(),
                //swerveClass.functionThatStops
            )
        )
        return routine
    }

    fun bTaxi(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("bTaxi")
        val rTaxiTrajectory: AutoTrajectory = routine.trajectory("2s(b)")
        routine.active().onTrue(
            Commands.sequence(
                rTaxiTrajectory.resetOdometry(),
                rTaxiTrajectory.cmd()
                //swerveClass.functionThatStops
            )
        )
        return routine
    }

    fun positionOneRed(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine ("positionOneRed")
        val pOneRedTrajectory: AutoTrajectory = routine.trajectory("1s(r)")
        routine.active().onTrue(
            Commands.sequence(
                //conveyorClass.functionThatShoots
                pOneRedTrajectory.resetOdometry(),
                pOneRedTrajectory.cmd())
                //swerveClass.functionThatStops
        )
        return routine
    }

    fun positionTwoRed(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("positionTwoRed")
        val pTwoRedTrajectory: AutoTrajectory = routine.trajectory("2s(r)")
        routine.active().onTrue(
            Commands.sequence(
                //conveyorClass.functionThatShoots
                pTwoRedTrajectory.resetOdometry(),
                pTwoRedTrajectory.cmd()
                //swerveClass.functionThatStops
            )
        )
        return routine
    }

    fun rTaxi(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("positionTwoRedShooterFail")
        val rTaxiTrajectory: AutoTrajectory = routine.trajectory("2s(r)")
        routine.active().onTrue(
            Commands.sequence(
                rTaxiTrajectory.resetOdometry(),
                rTaxiTrajectory.cmd()
                //swerveClass.functionThatStops
            )
        )
        return routine
    }
    fun justShoot(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("justShoot")
        routine.active().onTrue(
            Commands.sequence(
                //shooterClass.functionThatShoots
            )
        )
        return routine
    }
    fun doNothing(): AutoRoutine {
        val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
        return nothing
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("P1 blue taxi", this::positionOneBlue)
        autoChooser.addRoutine("P2 blue taxi", this::positionTwoBlue)
        autoChooser.addRoutine("just blue taxi", this::bTaxi)
        autoChooser.addRoutine("P1 red taxi", this::positionOneRed)
        autoChooser.addRoutine("P2 red taxi", this::positionTwoRed)
        autoChooser.addRoutine("just red taxi", this::rTaxi)
        autoChooser.addRoutine("just shoot", this::justShoot)
        autoChooser.addRoutine("do nothing", this::doNothing)
    }
}


