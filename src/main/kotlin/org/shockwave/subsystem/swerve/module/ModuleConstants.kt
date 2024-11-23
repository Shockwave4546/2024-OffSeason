package org.shockwave.subsystem.swerve.module

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

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
    private const val ROT_ENCODER_INVERTED = true
    private const val DRIVE_DIRECTION_INVERTED = true

    // Calculations required for driving motor conversion factors and feed forward
    private const val DRIVE_FREE_SPEED_RPS = MotorConstants.NEO_FREE_SPEED_RPM / 60
    private const val WHEEL_DIAMETER = 0.071
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    private const val DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15)
    private const val DRIVE_WHEEL_FREE_SPEED_RPS = ((DRIVE_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_REDUCTION)

    private const val DRIVE_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION // m
    private const val DRIVE_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0 // m/s

    private const val ROT_POSITION_FACTOR = 2 * Math.PI // rad
    private const val ROT_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 // rad/s

    private const val ROT_PID_MIN_INPUT = 0.0 // rad
    private const val ROT_PID_MAX_INPUT = 2 * Math.PI // rad

    private const val VOLTAGE_COMPENSATION = 12.0

    val DRIVE_PIDF = PIDFGains(0.15, 0.0, 0.02, 1 / DRIVE_WHEEL_FREE_SPEED_RPS)
    private const val DRIVE_MIN_OUTPUT = -1.0
    private const val DRIVE_MAX_OUTPUT = 1.0

    val ROT_PIDF = PIDFGains(0.5, 0.0, 0.0)
    private const val ROT_MIN_OUTPUT = -1.0
    private const val ROT_MAX_OUTPUT = 1.0

    val DRIVE_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      inverted(DRIVE_DIRECTION_INVERTED)
      idleMode(SparkBaseConfig.IdleMode.kBrake)
      voltageCompensation(VOLTAGE_COMPENSATION)

      encoder
        .positionConversionFactor(DRIVE_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_VELOCITY_FACTOR)

      closedLoop
        .pidf(DRIVE_PIDF.p, DRIVE_PIDF.i, DRIVE_PIDF.d, DRIVE_PIDF.ff)
        .outputRange(DRIVE_MIN_OUTPUT, DRIVE_MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    }

    val ROT_CONFIG = SparkMaxConfig().apply {
      smartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT)
      inverted(ROT_ENCODER_INVERTED)
      idleMode(SparkBaseConfig.IdleMode.kBrake)
      voltageCompensation(VOLTAGE_COMPENSATION)

      encoder
        .positionConversionFactor(ROT_POSITION_FACTOR)
        .velocityConversionFactor(ROT_VELOCITY_FACTOR)

      closedLoop
        .pidf(ROT_PIDF.p, ROT_PIDF.i, ROT_PIDF.d, ROT_PIDF.ff)
        .outputRange(ROT_MIN_OUTPUT, ROT_MAX_OUTPUT)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(ROT_PID_MIN_INPUT)
        .positionWrappingMaxInput(ROT_PID_MAX_INPUT)
    }
  }
}