package frc.robot

import au.grapplerobotics.LaserCan
import com.studica.frc.AHRS
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.commands.ExampleCommand
import frc.robot.subsystems.ExampleSubsystem
import frc.robot.commands.SwerveDriveCommand
import frc.robot.subsystems.SwerveDriveSubsytem
import frc.robot.subsystems.Conveyor

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    val conveyor = Conveyor(conveyorSensor = LaserCan(23))

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
    private val ahrs = AHRS(AHRS.NavXComType.kUSB1)
    val drive = SwerveDriveSubsytem(ahrs)
    val driveCommand = SwerveDriveCommand(drive,ahrs,driverController)
    init
    {
        configureBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        Trigger { ExampleSubsystem.exampleCondition() }.onTrue(ExampleCommand())

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        driverController.b().whileTrue(ExampleSubsystem.exampleMethodCommand())
        driveCommand.schedule()
        //hard
        driverController.rightTrigger().whileTrue(conveyor.move(5.0))
        driverController.leftTrigger().whileTrue(conveyor.move(-2.5)) // out take
        //sensor
        driverController.rightBumper().whileTrue(conveyor.runDetect(4.0))
        driverController.leftBumper().whileTrue(conveyor.stop())


    }


}