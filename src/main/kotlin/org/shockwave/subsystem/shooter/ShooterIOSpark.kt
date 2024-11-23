package org.shockwave.subsystem.shooter

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.BOTTOM_CONFIG
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.BOT_CAN_ID
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.TOP_CAN_ID
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.TOP_CONFIG
import org.shockwave.utils.PIDFGains

class ShooterIOSpark : ShooterIO {
  private val botMotor = SparkMax(BOT_CAN_ID, SparkLowLevel.MotorType.kBrushless)
  private val botEncoder = botMotor.encoder
  private val botPID = botMotor.closedLoopController

  private val topMotor = SparkMax(TOP_CAN_ID, SparkLowLevel.MotorType.kBrushless)
  private val topEncoder = topMotor.encoder
  private val topPID = topMotor.closedLoopController

  init {
    botMotor.configure(BOTTOM_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    topMotor.configure(TOP_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
  }

  override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
    inputs.bottomRPS = botEncoder.velocity
    inputs.bottomAppliedVolts = botMotor.appliedOutput * botMotor.busVoltage
    inputs.bottomCurrent = botMotor.outputCurrent
    inputs.bottomTemp = botMotor.motorTemperature

    inputs.topRPS = topEncoder.velocity
    inputs.topAppliedVolts = topMotor.appliedOutput * topMotor.busVoltage
    inputs.topCurrent = topMotor.outputCurrent
    inputs.topTemp = topMotor.motorTemperature
  }

  override fun setBottomVelocitySetpoint(rps: Double): REVLibError = botPID.setReference(rps, SparkBase.ControlType.kVelocity)

  override fun setBotPIDF(pidf: PIDFGains): REVLibError = botMotor.configure(
    SparkMaxConfig().apply(
      ClosedLoopConfig().pidf(pidf.p, pidf.i, pidf.d, pidf.ff)
    ),
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  override fun stopBot() = botMotor.stopMotor()

  override fun setTopVelocitySetpoint(rps: Double): REVLibError = topPID.setReference(rps, SparkBase.ControlType.kVelocity)

  override fun setTopPIDF(pidf: PIDFGains): REVLibError = topMotor.configure(
    SparkMaxConfig().apply(
      ClosedLoopConfig().pidf(pidf.p, pidf.i, pidf.d, pidf.ff)
    ),
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  override fun stopTop() = topMotor.stopMotor()
}