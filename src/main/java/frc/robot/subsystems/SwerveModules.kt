package frc.robot.subsystems

import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants


class SwerveModules(val drive: SparkMax, val turn:SparkMax,val  pidDrive: PIDController, val pidTurn: PIDController) {
    fun setState(moduleState: SwerveModuleState){
        drive.setVoltage(
            pidDrive.calculate(
                drive.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleState.speedMetersPerSecond
            )
        )
        turn.setVoltage(
            pidTurn.calculate(
                turn.encoder.position,
                moduleState.angle.rotations,
            )
        )
    }
    fun getState(): SwerveModuleState{
        return SwerveModuleState(
            drive.encoder.velocity/(60 * Constants.OperatorConstants.whellCircumference),
            Rotation2d(turn.encoder.position)
        )

    }
}