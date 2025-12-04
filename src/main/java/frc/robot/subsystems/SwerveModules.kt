package frc.robot.subsystems

import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.Constants
import kotlin.math.PI


@Logged
class SwerveModules(
    val drive: SparkMax,
    val turn: SparkMax,
    val pidDrive: PIDController,
    val pidTurn: PIDController,
    val encoder: DutyCycleEncoder,
    val offset: Double,
    val driveFeedforward : SimpleMotorFeedforward,
) {
    init {
        pidTurn.enableContinuousInput(0.0, 2 * PI)
    }
    val position: Double
        get() = MathUtil.inputModulus(
            encoder.get() - offset,
            0.0,
            1.0
            )

    fun setState(moduleState: SwerveModuleState){
        drive.setVoltage(
            pidDrive.calculate(
                drive.encoder.velocity,
                moduleState.speedMetersPerSecond
            ) + driveFeedforward.calculate(
                moduleState.speedMetersPerSecond
            )
        )
        turn.setVoltage(
            pidTurn.calculate(
                position,
                MathUtil.inputModulus(
                    moduleState.angle.rotations,
                    0.0,
                    1.0,
                ),
            )
        )
    }

    // TODO: Use DCMotor to simulate the module motors
    fun getState(): SwerveModuleState{
        return SwerveModuleState(
            drive.encoder.velocity/(60 * Constants.OperatorConstants.whellCircumference),
            Rotation2d(turn.encoder.position)
        )
    }
}