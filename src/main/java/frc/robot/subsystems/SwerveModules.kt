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
    val driveFeedforward: SimpleMotorFeedforward,
) {

    init {
        pidTurn.enableContinuousInput(0.0, 1.0)
    }

    val position: Double
        get() = encoder.get()

    fun setState(moduleState: SwerveModuleState) {
        moduleState.optimize(Rotation2d.fromRotations(position))

        moduleState.speedMetersPerSecond *= moduleState.angle.minus(Rotation2d.fromRotations(position)).cos

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

    fun getState(): SwerveModuleState {
        return SwerveModuleState(
            drive.encoder.velocity,
            Rotation2d.fromRotations(encoder.get())
        )
    }
}