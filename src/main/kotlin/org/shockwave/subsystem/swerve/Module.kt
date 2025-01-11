package org.shockwave.subsystem.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger

class Module(private val io: ModuleIO, private val position: ModulePosition) : ModuleIO {
  private val inputs = ModuleIO.ModuleIOInputs()
  private val driveDisconnectedAlert = Alert("${position.name} Module Drive Disconnected", Alert.AlertType.kError)
  private val turnDisconnectedAlert = Alert("${position.name} Module Turn Disconnected", Alert.AlertType.kError)
  private var odometryPositions = arrayOf<SwerveModulePosition>()

  fun periodic() {
    io.updateInputs(inputs)
    Logger.processInputs("Drive/Module/${position.name}", inputs)

    // Calculate positions for odometry
    val sampleCount = inputs.odometryTimestamps.size // All signals are sampled together
    odometryPositions = Array(sampleCount) { SwerveModulePosition() }
    for (i in 0..<sampleCount) {
      val positionMeters = inputs.odometryDrivePositionsMeters[i] * Units.inchesToMeters(1.5)
      val angle = inputs.odometryTurnPositions[i]
      odometryPositions[i] = SwerveModulePosition(positionMeters, angle)
    }

    driveDisconnectedAlert.set(!inputs.driveConnected)
    turnDisconnectedAlert.set(!inputs.turnConnected)
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it.  */
  fun runSetpoint(state: SwerveModuleState) {
    // Optimize velocity setpoint
    state.optimize(getAngle())
    state.cosineScale(inputs.turnPosition)

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / Units.inchesToMeters(1.5))
    io.setTurnPosition(state.angle)
  }

  /** Runs the module with the specified output while controlling to zero degrees.  */
  fun runCharacterization(voltage: Double) {
    io.setDriveVoltage(voltage)
    io.setTurnPosition(Rotation2d())
  }

  /** Disables all outputs to motors.  */
  fun stop() {
    io.setDriveVoltage(0.0)
    io.setTurnVoltage(0.0)
  }

  /** Returns the current turn angle of the module.  */
  fun getAngle() = inputs.turnPosition

  /** Returns the current drive position of the module in meters.  */
  fun getPositionMeters() = inputs.drivePositionRads * Units.inchesToMeters(1.5);

  /** Returns the current drive velocity of the module in meters per second.  */
  fun getVelocityMetersPerSec() = inputs.driveVelocityRadsPerSec * Units.inchesToMeters(1.5);

  /** Returns the current drive position of the module in rads */
  fun getWheelRadiusCharacterizationPosition() = inputs.drivePositionRads

  fun getFFCharacterizationVelocity() = inputs.driveVelocityRadsPerSec

  /** Returns the module position (turn angle and drive position).  */
  fun getPosition() = SwerveModulePosition(getPositionMeters(), getAngle())

  /** Returns the module state (turn angle and drive velocity).  */
  fun getState() = SwerveModuleState(getVelocityMetersPerSec(), getAngle())

  /** Returns the module positions received this cycle.  */
  fun getOdometryPositions() = odometryPositions

  /** Returns the timestamps of the samples received this cycle.  */
  fun getOdometryTimestamps() = inputs.odometryTimestamps
}