package org.shockwave.subsystem.intake

import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.shockwave.MotorConstants

class IntakeConstants {
  companion object {
    const val INTAKE_FEED_SECONDS = 0.5

    const val LIMIT_SWITCH_DIO_PORT = 0
    const val MOTOR_CAN_ID = 32

    val MOTOR_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT)
      idleMode(SparkBaseConfig.IdleMode.kBrake)
    }
  }
}