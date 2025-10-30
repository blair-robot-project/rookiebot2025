package frc.robot.subsystems
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import frc.robot.subsystems.constants.SwerveDriveConstants

class SwerveDriveSubsytem : SubsystemBase() {
    //These Numbers aren't real since we don't know the distances
    val m_kinematics = SwerveDriveKinematics(
        SwerveDriveConstants.m_frontLeftLocation,
        SwerveDriveConstants.m_frontRightLocation,
        SwerveDriveConstants.m_backLeftLocation,
        SwerveDriveConstants.m_backRightLocation
    )

}