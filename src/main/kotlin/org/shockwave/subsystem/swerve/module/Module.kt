package org.shockwave.subsystem.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.Logger
import org.shockwave.utils.TunablePIDF

class Module(private val module: ModuleIO, private val position: ModulePosition) {
  private val inputs = ModuleIO.ModuleIOInputs()
  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  private val drivePIDF = TunablePIDF("${position.key}/Drive/Tuning/", ModuleConstants.DRIVE_PIDF)
  private val rotPIDF = TunablePIDF("${position.key}/Rot/Tuning/", ModuleConstants.ROT_PIDF)

  init {
    module.setDesiredState(desiredState)

    drivePIDF.periodic(module::setDrivePIDF) { value ->
      module.setDesiredState(SwerveModuleState(value, module.getState().angle))
    }

    rotPIDF.periodic(module::setRotPIDF) { value ->
      module.setDesiredState(SwerveModuleState(module.getState().speedMetersPerSecond, Rotation2d(value)))
    }
  }

  fun periodic() {
    module.updateInputs(inputs)
    Logger.processInputs(position.key, inputs)
  }

  fun setDesiredState(desiredState: SwerveModuleState) {
    this.desiredState = desiredState
    module.setDesiredState(desiredState)
  }

  fun getDesiredState() = desiredState

  fun getState() = module.getState()

  fun getPosition() = module.getPosition()

  fun resetDriveEncoder() = module.resetDriveEncoder()
}