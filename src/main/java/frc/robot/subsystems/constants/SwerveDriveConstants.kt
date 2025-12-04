package frc.robot.subsystems.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

object SwerveDriveConstants {
    val m_frontLeftLocation = Translation2d(0.276225,0.276225)
    val m_frontRightLocation = Translation2d(-0.276225,0.276225)
    val m_backLeftLocation = Translation2d(0.276225,-0.276225)
    val m_backRightLocation = Translation2d(-0.276225,-0.276225)

    const val maxVelocity = 3.0

    const val Drive_Gearing = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)

    const val frontLeftMotorID = 30
    const val frontLeftTurnMotorID = 8
    const val frontLeftAbsoluteEncoderID = 8
    val frontLeftOffset = Units.radiansToRotations(2.871)
    const val frontRightMotorID = 9
    const val frontRightTurnMotorID = 10
    const val frontRightAbsoluteEncoderID = 7
    val frontRightOffset = Units.radiansToRotations(-2.294)
    const val backLeftMotorID = 11
    const val backLeftTurnMotorID = 12
    const val backLeftAbsoluteEncoderID = 6
    val backLeftOffset = Units.radiansToRotations(1.290)
    const val backRightMotorID = 5
    const val backRightTurnMotorID = 62
    const val backRightAbsoluteEncoderID = 9
    val backRightOffset = Units.radiansToRotations(0.815)

    // All lines before aren't fnialized
    const val kp = 1.0
    const val ki = 0.0
    const val kd = 0.0

    const val turnKP = 1.0
    const val turnKI = 0.0
    const val turnKD = 0.0


    const val kS = 1.0
    const val kV = 1.0
    const val kA = 1.0
 }