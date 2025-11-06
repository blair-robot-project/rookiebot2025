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
    val fltController = PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    val frController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val frtController = PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    val blController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val bltController = PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    val brController = PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd)
    val brtController = PIDController(SwerveDriveConstants.turnKP,SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    val m_kinematics = SwerveDriveKinematics(
        SwerveDriveConstants.m_frontLeftLocation,
        SwerveDriveConstants.m_frontRightLocation,
        SwerveDriveConstants.m_backLeftLocation,
        SwerveDriveConstants.m_backRightLocation
    )
    val driveMotorConfig = SparkMaxConfig()
    val turnMotorConfig = SparkMaxConfig()
    val frontLeftMotor = SparkMax(SwerveDriveConstants.frontLeftMotorID, SparkLowLevel.MotorType.kBrushless)
    val frontLeftTurnMotor = SparkMax(SwerveDriveConstants.frontLeftTurnMotorID, SparkLowLevel.MotorType.kBrushless)
    val frontRightMotor = SparkMax(SwerveDriveConstants.frontRightMotorID,SparkLowLevel.MotorType.kBrushless)
    val frontRightTurnMotor = SparkMax(SwerveDriveConstants.frontRightTurnMotorID,SparkLowLevel.MotorType.kBrushless)
    val backLeftMotor = SparkMax(SwerveDriveConstants.backLeftMotorID, SparkLowLevel.MotorType.kBrushless)
    val backLeftTurnMotor = SparkMax(SwerveDriveConstants.backLeftTurnMotorID, SparkLowLevel.MotorType.kBrushless)
    val backRightMotor = SparkMax(SwerveDriveConstants.backRightMotorID, SparkLowLevel.MotorType.kBrushless)
    val backRightTurnMotor = SparkMax(SwerveDriveConstants.backRightTurnMotorID, SparkLowLevel.MotorType.kBrushless)

    var desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0,0.0, ahrs.rotation2d)
    init {

        driveMotorConfig.smartCurrentLimit(80)
        driveMotorConfig.inverted(false)
        driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
        frontLeftMotor.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        frontRightMotor.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backLeftMotor.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backRightMotor.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        turnMotorConfig.smartCurrentLimit(80)
        turnMotorConfig.inverted(false)
        turnMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

        frontLeftTurnMotor.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        frontRightTurnMotor.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backLeftTurnMotor.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backRightTurnMotor.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }
    fun setSpeeds(x: Double, y: Double, omega: Double){
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,omega, ahrs.rotation2d)
        val moduleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds)
        frontLeftMotor.setVoltage(
            flController.calculate(
                frontLeftMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[0].speedMetersPerSecond
            )
        )
        frontLeftTurnMotor.setVoltage(
            fltController.calculate(
                frontLeftTurnMotor.encoder.position,
                moduleStates[0].angle.rotations,
            )
        )
        frontRightMotor.setVoltage(
            frController.calculate(
                frontRightMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[1].speedMetersPerSecond
            )
        )
        frontRightTurnMotor.setVoltage(
            frtController.calculate(
                frontRightTurnMotor.encoder.position,
                moduleStates[1].angle.rotations,
            )
        )
        backLeftMotor.setVoltage(
            blController.calculate(
                backLeftMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[2].speedMetersPerSecond
            )
        )
        backLeftTurnMotor.setVoltage(
            bltController.calculate(
                backLeftTurnMotor.encoder.position,
                moduleStates[2].angle.rotations,
            )
        )
        backRightMotor.setVoltage(
            brController.calculate(
                backRightMotor.encoder.velocity / (60 * Constants.OperatorConstants.whellCircumference),
                moduleStates[3].speedMetersPerSecond
            )
        )

        backRightTurnMotor.setVoltage(
            brtController.calculate(
                backRightTurnMotor.encoder.position,
                moduleStates[3].angle.rotations,
            )
        )

    }
}