package org.shockwave.subsystem.shooterpivot

import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.Alert
import org.shockwave.MotorConstants
import org.shockwave.utils.InterpolatingDoubleTreeMap
import org.shockwave.utils.PIDFGains

class ShooterPivotConstants {
  companion object {
    const val MOTOR_CAN_ID = 36
    const val ANGLE_CONVERSION_FACTOR = 360.0

    val GAINS = PIDFGains(0.03, 0.0, 0.007)

    private const val ENCODER_INVERTED = false
    private const val MIN_OUTPUT = -1.0
    private const val MAX_OUTPUT = 1.0
    const val ANGLE_TOLERANCE = 2.0 // degrees

    const val ANGLE_OFFSET = 291.0 // degrees

    const val MIN_ANGLE = 2.5 // degrees
    const val MAX_ANGLE = 80.0 // degrees

    val SHOULD_STOP_MOTOR_ALERT = Alert("The encoder is reporting an angle that will break the pivot", Alert.AlertType.kError)

    val MOTOR_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      idleMode(SparkBaseConfig.IdleMode.kBrake)

      absoluteEncoder
        .inverted(ENCODER_INVERTED)
        .zeroOffset(ANGLE_OFFSET)
        .positionConversionFactor(ANGLE_CONVERSION_FACTOR)

      closedLoop
        .pidf(GAINS.p, GAINS.i, GAINS.d, GAINS.ff)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    }

    val ANGLE_PREDICTOR = InterpolatingDoubleTreeMap(
      mapOf(
        1.5 to 25.0,
        1.7 to 28.0,
        1.9 to 32.0,
        2.1 to 36.0,
        2.3 to 38.0,
        2.5 to 42.0,
        2.7 to 42.0,
        2.9 to 43.0
      )
    )
  }
}