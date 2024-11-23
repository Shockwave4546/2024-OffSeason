package org.shockwave.subsystem.swerve.module

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.shockwave.subsystem.swerve.module.ModuleConstants.Companion.DRIVE_CONFIG
import org.shockwave.subsystem.swerve.module.ModuleConstants.Companion.ROT_CONFIG
import org.shockwave.utils.PIDFGains

class ModuleIOSpark(driveID: Int, rotID: Int, private val position: ModulePosition) : ModuleIO {
  private val driveMotor = SparkMax(driveID, SparkLowLevel.MotorType.kBrushless)
  private val driveEncoder = driveMotor.encoder
  private val drivePID = driveMotor.closedLoopController

  private val rotMotor = SparkMax(rotID, SparkLowLevel.MotorType.kBrushless)
  private val rotEncoder = rotMotor.absoluteEncoder
  private val rotPID = rotMotor.closedLoopController

  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  init {
    driveMotor.configure(DRIVE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    rotMotor.configure(ROT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)

    desiredState.angle = Rotation2d(0.0)
    resetDriveEncoder()
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    inputs.drivePosition = driveEncoder.position
    inputs.driveVelocity = driveEncoder.velocity
    inputs.driveAppliedVolts = driveMotor.appliedOutput * driveMotor.busVoltage
    inputs.driveCurrent = driveMotor.outputCurrent
    inputs.driveTemp = driveMotor.motorTemperature

    inputs.rotPosition = Rotation2d(rotEncoder.position)
    inputs.rotVelocity = rotEncoder.velocity
    inputs.rotAppliedVolts = rotMotor.appliedOutput * rotMotor.busVoltage
    inputs.rotCurrent = rotMotor.outputCurrent
    inputs.rotTemp = rotMotor.motorTemperature
  }

  /**
   * Returns the current state of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current state of the module.
   */
  override fun getState() = SwerveModuleState(driveEncoder.velocity, Rotation2d(rotEncoder.getPosition() - position.angleOffset))

  override fun getPosition() = SwerveModulePosition(driveEncoder.position, Rotation2d(rotEncoder.getPosition() - position.angleOffset))

  override fun setDesiredState(desiredState: SwerveModuleState) {
    // Apply chassis angular offset to the desired state.
    val offsetDesiredState = SwerveModuleState(
      desiredState.speedMetersPerSecond,
      desiredState.angle.plus(Rotation2d.fromRadians(position.angleOffset))
    )

    // Optimize the reference state to avoid spinning further than 90 degrees.
    offsetDesiredState.optimize(Rotation2d(rotEncoder.getPosition()))

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.setReference(offsetDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity)
    rotPID.setReference(offsetDesiredState.angle.radians, SparkBase.ControlType.kPosition)

    this.desiredState = desiredState
  }

  override fun resetDriveEncoder(): REVLibError = driveEncoder.setPosition(0.0)

  override fun setDrivePIDF(pidf: PIDFGains): REVLibError = driveMotor.configure(
    SparkMaxConfig().apply(
      ClosedLoopConfig().pidf(pidf.p, pidf.i, pidf.d, pidf.ff)
    ),
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  override fun setRotPIDF(pidf: PIDFGains): REVLibError = rotMotor.configure(
    SparkMaxConfig().apply(
      ClosedLoopConfig().pidf(pidf.p, pidf.i, pidf.d, pidf.ff)
    ),
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )
}