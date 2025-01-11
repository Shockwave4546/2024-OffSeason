package org.shockwave.subsystem.swerve

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import org.shockwave.utils.SPARK_STICKY_FAULT
import org.shockwave.utils.ifOk
import org.shockwave.utils.tryUntilOk
import java.util.function.DoubleSupplier

class ModuleIOSpark(private val position: ModulePosition) : ModuleIO {
  private val driveSpark = SparkMax(position.driveID, SparkLowLevel.MotorType.kBrushless)
  private val driveEncoder = driveSpark.encoder
  private val drivePID = driveSpark.closedLoopController

  private val turnSpark = SparkMax(position.turnID, SparkLowLevel.MotorType.kBrushless)
  private val turnEncoder = turnSpark.absoluteEncoder
  private val turnPID = turnSpark.closedLoopController

  private val timestampQueue = SparkOdometryThread.makeTimestampQueue()
  private val drivePositionQueue = SparkOdometryThread.registerSignal(driveSpark, driveEncoder::getPosition)
  private val turnPositionQueue = SparkOdometryThread.registerSignal(turnSpark, turnEncoder::getPosition)

  private val driveConnectedDebounce = Debouncer(0.5)
  private val turnConnectedDebounce = Debouncer(0.5)

  init {
    driveSpark.tryUntilOk(5) {
      it.configure(ModuleConstants.DRIVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    turnSpark.tryUntilOk(5) {
      it.configure(ModuleConstants.TURN_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    turnSpark.tryUntilOk(5) {
      it.encoder.setPosition(0.0)
    }
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    SPARK_STICKY_FAULT = false
    driveSpark.ifOk(driveEncoder::getPosition) { inputs.drivePositionRads = it }
    driveSpark.ifOk(driveEncoder::getVelocity) { inputs.driveVelocityRadsPerSec = it }
    driveSpark.ifOk(
      arrayOf(DoubleSupplier { driveSpark.appliedOutput }, DoubleSupplier { driveSpark.busVoltage })
    ) { values -> inputs.driveAppliedVolts = values[0] * values[1] }
    driveSpark.ifOk({ driveSpark.outputCurrent }, { inputs.driveCurrentAmps = it })
    inputs.driveConnected = driveConnectedDebounce.calculate(!SPARK_STICKY_FAULT)

    SPARK_STICKY_FAULT = false
    turnSpark.ifOk(
      { turnEncoder.position },
      { value -> inputs.turnPosition = Rotation2d(value).minus(Rotation2d.fromRadians(position.angleOffset)) })
    turnSpark.ifOk({ turnEncoder.velocity }, { inputs.turnVelocityRadPerSec = it })
    turnSpark.ifOk(
      arrayOf(DoubleSupplier { turnSpark.appliedOutput }, DoubleSupplier { turnSpark.busVoltage })
    ) { values -> inputs.turnAppliedVolts = values[0] * values[1] }
    turnSpark.ifOk({ turnSpark.outputCurrent }, { inputs.turnCurrentAmps = it })
    inputs.turnConnected = turnConnectedDebounce.calculate(!SPARK_STICKY_FAULT)

    inputs.odometryTimestamps = timestampQueue.map { it.toDouble() }.toDoubleArray()
    inputs.odometryDrivePositionsMeters = drivePositionQueue.map { it.toDouble() }.toDoubleArray()
    inputs.odometryTurnPositions = turnPositionQueue.map { Rotation2d(it).minus(Rotation2d.fromRadians(position.angleOffset)) }.toTypedArray()
    timestampQueue.clear()
    drivePositionQueue.clear()
    turnPositionQueue.clear()
  }

  override fun setDriveVoltage(voltage: Double) = driveSpark.set(voltage)

  override fun setTurnVoltage(voltage: Double) = turnSpark.set(voltage)

  override fun setDriveVelocity(metersPerSecond: Double) {
    /**
     *     double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
     *     driveController.setReference(
     *         velocityRadPerSec, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
     */

    drivePID.setReference(metersPerSecond, SparkBase.ControlType.kVelocity)
  }

  override fun setTurnPosition(rotation: Rotation2d) {
    // Applies
    val setpoint = MathUtil.inputModulus(rotation.plus(Rotation2d.fromRadians(position.angleOffset)).radians, ModuleConstants.TURN_PID_MIN_INPUT, ModuleConstants.TURN_PID_MAX_INPUT)
    turnPID.setReference(setpoint, SparkBase.ControlType.kPosition)
  }
}