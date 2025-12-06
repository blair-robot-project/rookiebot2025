package frc.robot.subsystems.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object SwerveDriveConstants {
    val m_frontLeftLocation = Translation2d(0.276225,0.276225)
    val m_frontRightLocation = Translation2d(0.276225,-0.276225)
    val m_backLeftLocation = Translation2d(-0.276225,0.276225)
    val m_backRightLocation = Translation2d(-0.276225,-0.276225)

    const val maxVelocity = 3.0 // m/s
    const val maxRotationalSpeed = 2 * PI // rad/s

    const val Drive_Gearing = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) // L3

    const val frontLeftMotorID = 30
    const val frontLeftTurnMotorID = 8
    const val frontLeftAbsoluteEncoderID = 8
    val frontLeftOffset = 0.953

    const val frontRightMotorID = 9
    const val frontRightTurnMotorID = 10
    const val frontRightAbsoluteEncoderID = 7
    val frontRightOffset = 0.131

    const val backLeftMotorID = 11
    const val backLeftTurnMotorID = 12
    const val backLeftAbsoluteEncoderID = 6
    val backLeftOffset = 0.710

    const val backRightMotorID = 5
    const val backRightTurnMotorID = 62
    const val backRightAbsoluteEncoderID = 9
    val backRightOffset = 0.865

    // all lines before aren't finalized
    const val kp = 0.75
    const val ki = 0.0
    const val kd = 0.0

    const val turnKP = 3.0
    const val turnKI = 0.0
    const val turnKD = 0.0

    const val kS = 0.15
    const val kV = 2.36
    const val kA = 0.33
 }