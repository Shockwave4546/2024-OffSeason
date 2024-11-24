package org.shockwave.subsystem.intakepivot

import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.Alert
import org.shockwave.MotorConstants
import org.shockwave.utils.PIDFGains

class IntakePivotConstants {
  companion object {
    private const val ENCODER_INVERTED = false
    const val MOTOR_CAN_ID = 33
    const val ANGLE_CONVERSION_FACTOR = 360.0

    val GAINS = PIDFGains(0.008, 0.0, 0.0)
    private const val MIN_OUTPUT = -1.0
    private const val MAX_OUTPUT = 1.0

    const val ANGLE_TOLERANCE = 3.0 // degrees

    const val ANGLE_OFFSET = 64.0 // degrees
    const val MIN_ANGLE = 2.5 // degrees
    const val MAX_ANGLE = 205.0 // degrees

    val SHOULD_STOP_MOTOR_ALERT = Alert("The encoder is reporting an angle that will break the pivot", Alert.AlertType.kError)

    val MOTOR_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      idleMode(SparkBaseConfig.IdleMode.kBrake)

      absoluteEncoder
        .positionConversionFactor(ANGLE_CONVERSION_FACTOR)
        .zeroOffset(ANGLE_OFFSET)
        .inverted(ENCODER_INVERTED)

      closedLoop
        .pidf(GAINS.p, GAINS.i, GAINS.d, GAINS.ff)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    }
  }
}