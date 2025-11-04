package frc.robot.subsystems.constants

import edu.wpi.first.math.geometry.Translation2d

object SwerveDriveConstants {
    //NUMS NOT FINALIZED.
    val m_frontLeftLocation = Translation2d(0.351,0.351)
    val m_frontRightLocation = Translation2d(-0.351,0.351)
    val m_backLeftLocation = Translation2d(0.351,-0.351)
    val m_backRightLocation = Translation2d(-0.351,0.351)

    const val frontLeftMotorID = 1
    const val frontLeftTurnMotorID = 5
    const val frontRightMotorID = 2
    const val frontRightTurnMotorID = 6
    const val backLeftMotorID = 3
    const val backLeftTurnMotorID = 7
    const val backRightMotorID = 4
    const val backRightTurnMotorID = 8

    const val kp = 1.0
    const val ki = 1.0
    const val kd = 1.0

    const val turnKP = 1.0
    const val turnKI = 1.0
    const val turnKD = 1.0
}