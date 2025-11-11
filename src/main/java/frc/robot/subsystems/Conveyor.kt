package frc.robot.subsystems.conveyor

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.Conveyor.ConveyorConstants
import frc.robot.subsystems.Conveyor.ConveyorConstants.CONVEYOR_ID
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.LaserCan
import au.grapplerobotics.simulation.MockLaserCan

class Conveyor (
    private val conveyorSensor: LaserCanInterface,
)   : SubsystemBase(){
    private val sensors =
        listOf(
            conveyorSensor
        )
    private var allSensorsConfigured = true
    private var lasercanConfigured = listOf<Boolean>()
    val conveyor = SparkMax(ConveyorConstants.CONVEYOR_ID, SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()
    val conveyor2 = SparkMax(ConveyorConstants.CONVEYOR_ID_2, SparkLowLevel.MotorType.kBrushless)
    val config2 = SparkMaxConfig()
    init {
        try {
            conveyorSensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
            for (sensor in sensors) {
                if (sensor != conveyorSensor) {
                    sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS)
                }
                sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
                sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
                lasercanConfigured.plus(true)
            }
        } catch (_: Exception) {
            lasercanConfigured.plus(false)
            allSensorsConfigured = false
        }
        config.smartCurrentLimit(ConveyorConstants.CURRENT_LIMIT)
        config2.smartCurrentLimit(ConveyorConstants.CURRENT_LIMIT)
        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
        config2.idleMode(SparkBaseConfig.IdleMode.kBrake)
        config2.follow(conveyor, true)
        conveyor2.configure(config2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
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
    private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
        val measurement = laserCan.measurement
        return measurement != null && (
                measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
                        measurement.distance_mm <= ConveyorConstants.CONVEYOR_DETECTION_THRESHOLD

                )
    }
    fun footballDetected(): Boolean = laserCanDetected(conveyorSensor)

    fun footBallNotDetected(): Boolean = !footballDetected()
}