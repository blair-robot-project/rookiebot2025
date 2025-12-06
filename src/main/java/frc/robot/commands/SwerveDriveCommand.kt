package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveDrive
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.constants.SwerveDriveConstants

class SwerveDriveCommand(
    val drive: SwerveDrive,
    val driverController: CommandXboxController
): Command() {

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val xDead = MathUtil.applyDeadband(driverController.leftY,0.1)
        val yDead = MathUtil.applyDeadband(driverController.leftX,0.1)

        drive.setSpeeds(
            xDead * SwerveDriveConstants.maxVelocity,
            yDead * SwerveDriveConstants.maxVelocity,
            driverController.rightX * SwerveDriveConstants.maxRotationalSpeed
        )
    }

}