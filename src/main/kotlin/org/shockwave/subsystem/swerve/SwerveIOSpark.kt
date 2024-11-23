package org.shockwave.subsystem.swerve

import org.shockwave.subsystem.swerve.module.Module
import org.shockwave.subsystem.swerve.module.ModuleIOSpark
import org.shockwave.subsystem.swerve.module.ModulePosition

class SwerveIOSpark : SwerveIO {
  override val frontLeft = Module(
    ModuleIOSpark(
      SwerveConstants.FRONT_LEFT_DRIVING_CAN_ID,
      SwerveConstants.FRONT_LEFT_TURNING_CAN_ID,
      ModulePosition.FRONT_LEFT
    ),
    ModulePosition.FRONT_LEFT
  )

  override val frontRight = Module(
    ModuleIOSpark(
      SwerveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
      SwerveConstants.FRONT_RIGHT_TURNING_CAN_ID,
      ModulePosition.FRONT_RIGHT
    ),
    ModulePosition.FRONT_RIGHT
  )

  override val backLeft = Module(
    ModuleIOSpark(
      SwerveConstants.BACK_LEFT_DRIVING_CAN_ID,
      SwerveConstants.BACK_LEFT_TURNING_CAN_ID,
      ModulePosition.BACK_LEFT
    ),
    ModulePosition.BACK_LEFT
  )

  override val backRight = Module(
    ModuleIOSpark(
      SwerveConstants.BACK_RIGHT_DRIVING_CAN_ID,
      SwerveConstants.BACK_RIGHT_TURNING_CAN_ID,
      ModulePosition.BACK_RIGHT
    ),
    ModulePosition.BACK_RIGHT
  )
}