package org.shockwave.subsystem.shooter

import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.shockwave.MotorConstants
import org.shockwave.MotorConstants.Companion.NEO_FREE_SPEED_RPM
import org.shockwave.utils.InterpolatingDoubleTreeMap
import org.shockwave.utils.PIDFGains

class ShooterConstants {
  companion object {
    const val BOT_CAN_ID = 31
    val BOT_GAINS = PIDFGains(0.03, 0.0, 0.0, 0.011)
    private const val BOT_INVERTED = true

    const val TOP_CAN_ID = 30
    val TOP_GAINS = PIDFGains(0.03, 0.0, 0.0, 0.011)
    private const val TOP_INVERTED = true

    private const val REV_CONVERSION_FACTOR = 1.0
    private const val RPS_CONVERSION_FACTOR = 1.0 / 60.0
    private const val MIN_OUTPUT = -1.0
    private const val MAX_OUTPUT = 1.0
    const val RPS_TOLERANCE = 2.5
    const val MIN_RPS = 0.0
    const val MAX_RPS = NEO_FREE_SPEED_RPM / 60.0

    val BOTTOM_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      inverted(BOT_INVERTED)
      idleMode(SparkBaseConfig.IdleMode.kBrake)

      encoder
        .positionConversionFactor(REV_CONVERSION_FACTOR)
        .velocityConversionFactor(RPS_CONVERSION_FACTOR)

      closedLoop
        .pidf(BOT_GAINS.p, BOT_GAINS.i, BOT_GAINS.d, BOT_GAINS.ff)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    }

    val TOP_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      inverted(TOP_INVERTED)
      idleMode(SparkBaseConfig.IdleMode.kBrake)

      encoder
        .positionConversionFactor(REV_CONVERSION_FACTOR)
        .velocityConversionFactor(RPS_CONVERSION_FACTOR)

      closedLoop
        .pidf(TOP_GAINS.p, TOP_GAINS.i, TOP_GAINS.d, TOP_GAINS.ff)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    }

    val BOT_RPS_PREDICTOR = InterpolatingDoubleTreeMap(
      mapOf(
        1.5 to 60.0,
        1.7 to 60.0,
        1.9 to 55.0,
        2.1 to 55.0,
        2.3 to 55.0,
        2.5 to 55.0,
        2.7 to 50.0,
        2.9 to 50.0
      )
    )

    val TOP_RPS_PREDICTOR = InterpolatingDoubleTreeMap(
      mapOf(
        1.5 to 30.0,
        1.7 to 40.0,
        1.9 to 45.0,
        2.1 to 45.0,
        2.3 to 45.0,
        2.5 to 50.0,
        2.7 to 55.0,
        2.9 to 55.0
      )
    )
  }
}