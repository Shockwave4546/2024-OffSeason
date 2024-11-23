package org.shockwave

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.shockwave.subsystem.intake.IntakeIOSpark
import org.shockwave.subsystem.intake.IntakeSubsystem
import org.shockwave.subsystem.shooter.ShooterIOSpark
import org.shockwave.subsystem.shooter.ShooterSubsystem
import org.shockwave.subsystem.swerve.SwerveIOSpark
import org.shockwave.subsystem.swerve.SwerveSubsystem
import org.shockwave.subsystem.swerve.commands.ResetFieldCentricDriveCommand
import org.shockwave.subsystem.swerve.commands.SetMaxSpeedCommand
import org.shockwave.subsystem.swerve.gyro.GyroIONavX
import org.shockwave.subsystem.vision.VisionIOReal
import org.shockwave.subsystem.vision.VisionSubsystem

object RobotContainer {
  val swerve: SwerveSubsystem?
  val vision: VisionSubsystem?

  //  val wrist: ShooterWristSubsystem?
//  val arm: IntakeArmSubsystem?
  val shooter: ShooterSubsystem?
  val intake: IntakeSubsystem?
//  val led: LEDSubsystem?

  val driverController = CommandXboxController(GlobalConstants.DRIVER_CONTROLLER_PORT)
  val operatorController = CommandXboxController(GlobalConstants.OPERATOR_CONTROLLER_PORT)
//  val autoManager: AutoManager?

  init {
    when (GlobalConstants.ROBOT_TYPE) {
      RobotType.REAL -> {
        swerve = SwerveSubsystem(SwerveIOSpark(), GyroIONavX())
        vision = VisionSubsystem(VisionIOReal(), swerve)
//        wrist = ShooterWristSubsystem(ShooterWristIOSpark(WristConstants.MOTOR_CAN_ID), vision)
//        arm = IntakeArmSubsystem(IntakeArmIOSpark(IntakeArmConstants.MOTOR_CAN_ID))
        shooter = ShooterSubsystem(ShooterIOSpark(), vision)
        intake = IntakeSubsystem(IntakeIOSpark())
//        led = LEDSubsystem()
//        autoManager = AutoManager(swerve, shooter, wrist, arm, intake, vision)
      }

      RobotType.SIM -> {
        swerve = null
        vision = null
//        wrist = null
//        arm = null
        shooter = null
        intake = null
//        led = null
//        autoManager = null
      }
    }

    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureBindings() {
    driverController.a().onTrue(InstantCommand({ swerve!!.toggleAutoAlign() }))
    driverController.b().onTrue(ResetFieldCentricDriveCommand(swerve!!, vision!!))
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