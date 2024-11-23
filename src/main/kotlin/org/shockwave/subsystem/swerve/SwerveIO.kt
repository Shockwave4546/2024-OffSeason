package org.shockwave.subsystem.swerve

import org.shockwave.subsystem.swerve.module.Module

interface SwerveIO {
  val frontLeft: Module
  val frontRight: Module
  val backLeft: Module
  val backRight: Module

  fun getModules() = arrayOf(frontLeft, frontRight, backLeft, backRight)
}