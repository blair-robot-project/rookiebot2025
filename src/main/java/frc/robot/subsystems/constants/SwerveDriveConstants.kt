package frc.robot.subsystems.constants

import edu.wpi.first.math.geometry.Translation2d

object SwerveDriveConstants {
    val m_frontLeftLocation = Translation2d(0.276225,0.276225)
    val m_frontRightLocation = Translation2d(-0.276225,0.276225)
    val m_backLeftLocation = Translation2d(0.276225,-0.276225)
    val m_backRightLocation = Translation2d(-0.276225,-0.276225)

    const val frontLeftMotorID = 6
    const val frontLeftTurnMotorID = 8
    const val frontRightMotorID = 9
    const val frontRightTurnMotorID = 10
    const val backLeftMotorID = 11
    const val backLeftTurnMotorID = 12
    const val backRightMotorID = 5
    const val backRightTurnMotorID = 62

    // ALL kp/ki/kd are not finalized
    const val kp = 1.0
    const val ki = 1.0
    const val kd = 1.0

    const val turnKP = 1.0
    const val turnKI = 1.0
    const val turnKD = 1.0
}