package frc.robot.subsystems.conveyor

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.Conveyor.ConveyorConstants
import frc.robot.subsystems.Conveyor.ConveyorConstants.CONVEYOR_ID

class Intake (
)   : SubsystemBase(){
    val conveyor = SparkMax(ConveyorConstants.CONVEYOR_ID, SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()
    init{
        config.smartCurrentLimit(ConveyorConstants.CURRENT_LIMIT)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        //config.inverted(True)
        conveyor.configure(config,SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }
    //Movement measured in voltage
    fun move(volt:Double){
        return conveyor.setVoltage(volt)
    }
    //Stops motor
    fun stop(){
        return conveyor.setVoltage(0.0)
    }
}