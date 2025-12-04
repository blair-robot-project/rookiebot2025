package frc.robot.commands
import com.studica.frc.AHRS
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveDrive
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.constants.SwerveDriveConstants

class SwerveDriveCommand(
    val drive: SwerveDrive,
    val ahrs: AHRS,
    val driverController: CommandXboxController
): Command(){

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.setSpeeds(
            driverController.leftX * SwerveDriveConstants.maxVelocity,
            driverController.leftY * SwerveDriveConstants.maxVelocity,
            driverController.rightX * SwerveDriveConstants.maxRotationalSpeed
        )
    }
}