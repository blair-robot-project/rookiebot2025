package frc.robot.commands
import com.studica.frc.AHRS
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveDriveSubsytem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class SwerveDriveCommand(
    val drive: SwerveDriveSubsytem,
    val ahrs: AHRS,
    val driverController: CommandXboxController
): Command(){

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.setSpeeds(driverController.leftY,driverController.leftX,driverController.rightX)
    }
}