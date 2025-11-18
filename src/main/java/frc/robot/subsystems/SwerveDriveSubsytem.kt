package frc.robot.subsystems
import com.fasterxml.jackson.annotation.Nulls
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.studica.frc.AHRS
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.constants.SwerveDriveConstants


class SwerveDriveSubsytem(
    val ahrs : AHRS
) : SubsystemBase() {
    val m_kinematics = SwerveDriveKinematics(
        SwerveDriveConstants.m_frontLeftLocation,
        SwerveDriveConstants.m_frontRightLocation,
        SwerveDriveConstants.m_backLeftLocation,
        SwerveDriveConstants.m_backRightLocation
    )

    val driveMotorConfig = SparkMaxConfig()
    val turnMotorConfig = SparkMaxConfig()
    //DRive and Turn
    val frontLeft = SwerveModules(
        SparkMax(SwerveDriveConstants.frontLeftMotorID, SparkLowLevel.MotorType.kBrushless),
        SparkMax(SwerveDriveConstants.frontLeftTurnMotorID, SparkLowLevel.MotorType.kBrushless),
        PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd),
        PIDController(SwerveDriveConstants.turnKP,SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    )
    val frontRight = SwerveModules(
        SparkMax(SwerveDriveConstants.frontRightMotorID,SparkLowLevel.MotorType.kBrushless),
        SparkMax(SwerveDriveConstants.frontRightTurnMotorID,SparkLowLevel.MotorType.kBrushless),
        PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd),
        PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    )
    val backLeft = SwerveModules(
        SparkMax(SwerveDriveConstants.backLeftMotorID,SparkLowLevel.MotorType.kBrushless),
        SparkMax(SwerveDriveConstants.backLeftTurnMotorID,SparkLowLevel.MotorType.kBrushless),
        PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd),
        PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    )
    val backRight = SwerveModules(
        SparkMax(SwerveDriveConstants.backRightMotorID,SparkLowLevel.MotorType.kBrushless),
        SparkMax(SwerveDriveConstants.backRightTurnMotorID,SparkLowLevel.MotorType.kBrushless),
        PIDController(SwerveDriveConstants.kp,SwerveDriveConstants.ki,SwerveDriveConstants.kd),
        PIDController(SwerveDriveConstants.turnKP, SwerveDriveConstants.turnKI, SwerveDriveConstants.turnKD)
    )

    var moduleStates : Array<SwerveModuleState> = arrayOf(SwerveModuleState(),SwerveModuleState(),SwerveModuleState(),SwerveModuleState())
    init {

        driveMotorConfig.smartCurrentLimit(80)
        driveMotorConfig.inverted(false)
        driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
        frontLeft.drive.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        frontRight.drive.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backLeft.drive.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backRight.drive.configure(
            driveMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        turnMotorConfig.smartCurrentLimit(80)
        turnMotorConfig.inverted(false)
        turnMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

        frontLeft.turn.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        frontRight.turn.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backLeft.turn.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        backRight.turn.configure(
            turnMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }
    fun setSpeeds(x: Double, y: Double, omega: Double) : Array<SwerveModuleState>{
        val desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,omega, ahrs.rotation2d)
        moduleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds)
        return moduleStates
    }

    override fun periodic() {
        frontLeft.setState(moduleStates[0])
        frontRight.setState(moduleStates[1])
        backLeft.setState(moduleStates[2])
        backRight.setState(moduleStates[3])
    }
    var robotPosition = Pose2d()
    override fun simulationPeriodic() {
        periodic()
        val robotSpeed: ChassisSpeeds = m_kinematics.toChassisSpeeds(frontLeft.getState(),frontRight.getState(),backLeft.getState(),backRight.getState())
        robotPosition=Pose2d(
        robotPosition.x+robotSpeed.vxMetersPerSecond*0.02,
        robotPosition.y+robotSpeed.vyMetersPerSecond*0.02,
        robotPosition.rotation+ Rotation2d(robotSpeed.omegaRadiansPerSecond*0.02)
        )
    }
    fun getRobotPosition() : Pose2d{
        return robotPosition
    }

}