package org.shockwave

import edu.wpi.first.wpilibj.DriverStation

object RobotContainer {
  init {
    configureBindings()
  }

  private fun configureBindings() {

  }

  fun isRedAlliance() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

  fun isCompMatch() = DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None
}