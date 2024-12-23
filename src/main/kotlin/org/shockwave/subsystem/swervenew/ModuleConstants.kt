package org.shockwave.subsystem.swervenew

import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.shockwave.MotorConstants
import org.shockwave.utils.PIDFGains

class ModuleConstants {
  companion object {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a robot that drives faster).
    private const val DRIVE_MOTOR_PINION_TEETH = 13

    private const val TURN_ENCODER_INVERTED = true // This must be true
    private const val DRIVE_MOTOR_INVERTED = true

    // Calculations required for driving motor conversion factors and feed forward
    private const val DRIVE_FREE_SPEED_RPS = MotorConstants.NEO_FREE_SPEED_RPM / 60
    private const val WHEEL_DIAMETER = 0.071
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    const val DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15)
    private const val DRIVE_WHEEL_FREE_SPEED_RPS = ((DRIVE_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_REDUCTION)

    private const val DRIVE_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION // m
    private const val DRIVE_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0 // m/s

    private const val TURN_POSITION_FACTOR = 2 * Math.PI // rad
    private const val TURN_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 // rad/s

    const val TURN_PID_MIN_INPUT = 0.0 // rad
    const val TURN_PID_MAX_INPUT = 2 * Math.PI // rad

    val DRIVE_PIDF = PIDFGains(0.15, 0.0, 0.02, 1 / DRIVE_WHEEL_FREE_SPEED_RPS)
    private const val DRIVE_MIN_OUTPUT = -1.0
    private const val DRIVE_MAX_OUTPUT = 1.0

    val TURN_PIDF = PIDFGains(0.5, 0.0, 0.0)
    private const val ROT_MIN_OUTPUT = -1.0
    private const val ROT_MAX_OUTPUT = 1.0

    val DRIVE_CONFIG = SparkMaxConfig().apply {
      inverted(DRIVE_MOTOR_INVERTED)
      idleMode(SparkBaseConfig.IdleMode.kBrake)
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      voltageCompensation(12.0)

      encoder
        .positionConversionFactor(DRIVE_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)

      closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(DRIVE_PIDF.p, DRIVE_PIDF.i, DRIVE_PIDF.d, DRIVE_PIDF.ff)

      signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((1000.0 / SwerveConstants.ODOMETRY_FREQUENCY).toInt())
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
    }

    val TURN_CONFIG = SparkMaxConfig().apply {
      idleMode(SparkBaseConfig.IdleMode.kBrake)
      smartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT)
      voltageCompensation(12.0)

      absoluteEncoder
        .inverted(TURN_ENCODER_INVERTED)
        .positionConversionFactor(TURN_POSITION_FACTOR)
        .velocityConversionFactor(TURN_VELOCITY_FACTOR)
        .averageDepth(2)

      closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(TURN_PID_MIN_INPUT, TURN_PID_MAX_INPUT)
        .pidf(TURN_PIDF.p, TURN_PIDF.i, TURN_PIDF.d, TURN_PIDF.ff)

      signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((1000.0 / SwerveConstants.ODOMETRY_FREQUENCY).toInt())
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
    }
  }
}