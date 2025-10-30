package frc.robot.subsystems
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.studica.frc.AHRS
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.constants.SwerveDriveConstants


class SwerveDriveSubsytem(
    val ahrs : AHRS
) : SubsystemBase() {
    val flController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val frController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val blController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val brController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)

    val m_kinematics = SwerveDriveKinematics(
        SwerveDriveConstants.m_frontLeftLocation,
        SwerveDriveConstants.m_frontRightLocation,
        SwerveDriveConstants.m_backLeftLocation,
        SwerveDriveConstants.m_backRightLocation
    )

    val frontLeftMotor = SparkMax(SwerveDriveConstants.frontLeftMotorID, SparkLowLevel.MotorType.kBrushless)
    val frontLeftMotorConfigure = SparkMaxConfig()
    val frontRightMotor = SparkMax(SwerveDriveConstants.frontRightMotorID,SparkLowLevel.MotorType.kBrushless)
    val frontRightMotorConfigure = SparkMaxConfig()
    val backLeftMotor = SparkMax(SwerveDriveConstants.backLeftMotorID, SparkLowLevel.MotorType.kBrushless)
    val backLeftMotorConfigure = SparkMaxConfig()
    val backRightMotor = SparkMax(SwerveDriveConstants.backRightMotorID, SparkLowLevel.MotorType.kBrushless)
    val backRightMotorConfigure = SparkMaxConfig()
    init {
        frontLeftMotorConfigure.smartCurrentLimit(80)
        frontLeftMotorConfigure.inverted(false)
        frontLeftMotorConfigure.idleMode(SparkBaseConfig.IdleMode.kBrake)
        frontLeftMotor.configure(frontRightMotorConfigure,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        frontRightMotorConfigure.smartCurrentLimit(80)
        frontRightMotorConfigure.inverted(false)
        frontRightMotorConfigure.idleMode(SparkBaseConfig.IdleMode.kBrake)
        frontRightMotor.configure(frontRightMotorConfigure,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        backLeftMotorConfigure.smartCurrentLimit(80)
        backLeftMotorConfigure.inverted(false)
        backLeftMotorConfigure.idleMode(SparkBaseConfig.IdleMode.kBrake)
        backLeftMotor.configure(backLeftMotorConfigure,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        backRightMotorConfigure.smartCurrentLimit(80)
        backRightMotorConfigure.inverted(false)
        backRightMotorConfigure.idleMode(SparkBaseConfig.IdleMode.kBrake)
        backRightMotor.configure(backRightMotorConfigure,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)
    }
    fun setSpeeds(x: Double, y: Double, omega: Double){
        val desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,omega, ahrs.rotation2d)
        val moduleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds)

        //val desiredWheelSpeeds = m_kinematics.toWheelSpeeds(desiredChassisSpeeds)
        frontLeftMotor.setVoltage(
            flController.calculate(
                frontLeftMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[0].speedMetersPerSecond
            )
        )
        frontRightMotor.setVoltage(
            frController.calculate(
                frontRightMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[1].speedMetersPerSecond
            )
        )
        backLeftMotor.setVoltage(
            blController.calculate(
                backLeftMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[2].speedMetersPerSecond
            )
        )
        backRightMotor.setVoltage(
            brController.calculate(
                backRightMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[3].speedMetersPerSecond
            )
        )


    }
}