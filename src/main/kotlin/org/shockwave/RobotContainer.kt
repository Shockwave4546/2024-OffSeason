package org.shockwave

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.shockwave.subsystem.swerve.*

object RobotContainer {
  val swerve: SwerveSubsystem

  private val driverController = CommandXboxController(GlobalConstants.DRIVER_CONTROLLER_PORT)

  val autoChooser: LoggedDashboardChooser<Command>

  init {
    when (GlobalConstants.CURRENT_MODE) {
      GlobalConstants.Companion.Mode.REAL -> {
        swerve = SwerveSubsystem(
          GyroIONavX(),
          ModuleIOSpark(ModulePosition.FRONT_LEFT),
          ModuleIOSpark(ModulePosition.FRONT_RIGHT),
          ModuleIOSpark(ModulePosition.BACK_LEFT),
          ModuleIOSpark(ModulePosition.BACK_RIGHT)
        )
      }

      // TODO: implement ModuleIOSim
      GlobalConstants.Companion.Mode.SIM -> {
        swerve = SwerveSubsystem(
          object : GyroIO {},
          object : ModuleIO {},
          object : ModuleIO {},
          object : ModuleIO {},
          object : ModuleIO {}
        )
      }

      GlobalConstants.Companion.Mode.REPLAY -> {
        swerve = SwerveSubsystem(
          object : GyroIO {},
          object : ModuleIO {},
          object : ModuleIO {},
          object : ModuleIO {},
          object : ModuleIO {}
        )
      }
    }

    autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

    // Set up SysId routines
    autoChooser.addOption("Swerve Wheel Radius Characterization", SwerveSubsystem.wheelRadiusCharacterization(swerve))
    autoChooser.addOption("Swerve Simple FF Characterization", SwerveSubsystem.feedforwardCharacterization(swerve))

    /// Implement for non comp match
//    // Set up SysId routines
//    autoChooser.addOption(
//      "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive)
//    )
//    autoChooser.addOption(
//      "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive)
//    )
//    autoChooser.addOption(
//      "Drive SysId (Quasistatic Forward)",
//      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
//    )
//    autoChooser.addOption(
//      "Drive SysId (Quasistatic Reverse)",
//      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
//    )
//    autoChooser.addOption(
//      "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
//    )
//    autoChooser.addOption(
//      "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
//    )

    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureBindings() {
    swerve.defaultCommand = SwerveSubsystem.joystickDrive(
      swerve,
      { -driverController.leftY },
      { -driverController.leftX },
      { -driverController.rightX })

    driverController.x().onTrue(Commands.runOnce(swerve::stopWithX, swerve))

    driverController.b().onTrue(
      Commands.runOnce(
        { swerve.setPose(Pose2d(swerve.getPose().translation, Rotation2d())) },
        swerve
      ).ignoringDisable(true)
    )

//    driverController.a().onTrue(InstantCommand({ swerve.toggleAutoAlign() }))
//    driverController.b().onTrue(ResetFieldCentricDriveCommand(swerve, vision))
//    driverController.x().onTrue(InstantCommand({ swerve.toggleX() }, swerve))
//    driverController.leftBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.2, 0.2))
//    driverController.rightBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.4, 0.4))

//    operatorController.povUp().onTrue(SetIntakeStateCommand(arm, ArmState.HOME))
//    operatorController.povDown().onTrue(SetIntakeStateCommand(arm, ArmState.FLOOR))
//    operatorController.povRight().onTrue(FeedShooterCommand(intake).withTimeout(0.25))
//
//    operatorController.leftBumper().onTrue(ResetRobotStateCommand(shooter, intake, arm, wrist))
//
//    operatorController.rightBumper().onTrue(FullSpitCommand(intake, shooter, arm, wrist))
//    operatorController.a().toggleOnTrue(FullShootCloseCommand(intake, shooter, arm, wrist))
//    operatorController.b().toggleOnTrue(AimAndShootCommand(intake, shooter, arm, wrist, swerve, vision))
//    operatorController.x().toggleOnTrue(FullShootAmpCommand(intake, shooter, arm, wrist))
//    operatorController.y().toggleOnTrue(FullIntakeCommand(arm, intake))
  }

  fun isRedAlliance() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

  fun isCompMatch() = DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None
}