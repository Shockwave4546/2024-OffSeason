package org.shockwave

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.shockwave.subsystem.intake.IntakeIOSpark
import org.shockwave.subsystem.intake.IntakeSubsystem
import org.shockwave.subsystem.intakepivot.IntakePivotIOSpark
import org.shockwave.subsystem.intakepivot.IntakePivotSubsystem
import org.shockwave.subsystem.shooter.ShooterIOSpark
import org.shockwave.subsystem.shooter.ShooterSubsystem
import org.shockwave.subsystem.shooterpivot.ShooterPivotIOSpark
import org.shockwave.subsystem.shooterpivot.ShooterPivotSubsystem
import org.shockwave.subsystem.swerve.SwerveIOSpark
import org.shockwave.subsystem.swerve.SwerveSubsystem
import org.shockwave.subsystem.swerve.commands.ResetFieldCentricDriveCommand
import org.shockwave.subsystem.swerve.commands.SetMaxSpeedCommand
import org.shockwave.subsystem.swerve.gyro.GyroIONavX
import org.shockwave.subsystem.vision.VisionIOReal
import org.shockwave.subsystem.vision.VisionSubsystem

object RobotContainer {
  val swerve = SwerveSubsystem(SwerveIOSpark(), GyroIONavX())
  val vision = VisionSubsystem(VisionIOReal(), swerve)

  val shooterPivot = ShooterPivotSubsystem(ShooterPivotIOSpark(), vision)
  val intakePivot = IntakePivotSubsystem(IntakePivotIOSpark())
  val shooter = ShooterSubsystem(ShooterIOSpark(), vision)
  val intake = IntakeSubsystem(IntakeIOSpark())
//  val led: LEDSubsystem?

  val driverController = CommandXboxController(GlobalConstants.DRIVER_CONTROLLER_PORT)
  val operatorController = CommandXboxController(GlobalConstants.OPERATOR_CONTROLLER_PORT)
//  val autoManager: AutoManager?

  init {
    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureBindings() {
    driverController.a().onTrue(InstantCommand({ swerve.toggleAutoAlign() }))
    driverController.b().onTrue(ResetFieldCentricDriveCommand(swerve, vision))
    driverController.x().onTrue(InstantCommand({ swerve.toggleX() }, swerve))
    driverController.leftBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.2, 0.2))
    driverController.rightBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.4, 0.4))

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