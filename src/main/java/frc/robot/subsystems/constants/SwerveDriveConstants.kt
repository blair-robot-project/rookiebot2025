package frc.robot.subsystems.constants

import edu.wpi.first.math.geometry.Translation2d

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
    const val frontRightMotorID = 9
    const val frontRightTurnMotorID = 10
    const val frontRightAbsoluteEncoderID = 7
    const val backLeftMotorID = 11
    const val backLeftTurnMotorID = 12
    const val backLeftAbsoluteEncoderID = 6
    const val backRightMotorID = 5
    const val backRightTurnMotorID = 62
    const val backRightAbsoluteEncoderID = 9

    // ALL kp/ki/kd are not finalized
    const val kp = 1.0
    const val ki = 0.0
    const val kd = 0.0

    const val turnKP = 1.0
    const val turnKI = 0.0
    const val turnKD = 0.0
}