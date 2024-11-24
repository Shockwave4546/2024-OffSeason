package org.shockwave.subsystem.swerve

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.shuffleboard.ShuffleboardBoolean
import org.shockwave.shuffleboard.ShuffleboardSpeed
import org.shockwave.subsystem.swerve.gyro.GyroIO
import org.shockwave.subsystem.swerve.module.Module
import org.shockwave.subsystem.vision.VisionConstants
import org.shockwave.utils.Tab
import org.shockwave.utils.TunablePIDF

class SwerveSubsystem(private val io: SwerveIO, private val gyro: GyroIO) : SubsystemBase() {
  private val isX = ShuffleboardBoolean(Tab.MATCH, "Is X?", false).withSize(3, 3)

  private val driveSpeedMultiplier = ShuffleboardSpeed(Tab.MATCH, "Drive Speed Multiplier", SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER).withSize(5, 2)
  private val rotSpeedMultiplier = ShuffleboardSpeed(Tab.MATCH, "Rot Speed Multiplier", SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER).withSize(5, 2)
  private val isFieldRelative = ShuffleboardBoolean(Tab.MATCH, "Is Field Relative?", true).withSize(3, 3)

  private val autoAlign = ShuffleboardBoolean(Tab.MATCH, "Auto Align", false).withSize(3, 3)

  private val headingController = ProfiledPIDController(
    SwerveConstants.HEADING_PIDF.p,
    SwerveConstants.HEADING_PIDF.i,
    SwerveConstants.HEADING_PIDF.d,
    TrapezoidProfile.Constraints(
      Units.degreesToRadians(540.0),
      Units.degreesToRadians(720.0)
    )
  )

  private val key = "Swerve"
  private val headingPIDF = TunablePIDF("$key/Tuning/Heading/", SwerveConstants.HEADING_PIDF)

  private val gyroInputs = GyroIO.GyroIOInputs()

  init {
    resetDriveEncoders()

    headingController.setTolerance(VisionConstants.HEADING_TOLERANCE)
  }

  override fun periodic() {
    io.getModules().forEach(Module::periodic)
    gyro.updateInputs(gyroInputs)
    Logger.processInputs("Gyro", gyroInputs)

    headingPIDF.periodic({ pidf ->
      headingController.p = pidf.p
      headingController.i = pidf.i
      headingController.d = pidf.d
    }, { value ->
      headingController.setGoal(value)
    })

    Logger.recordOutput("$key/(1) ModuleStates", *getEstimatedStates())
    Logger.recordOutput("$key/(2) DesiredStates", *getDesiredStates())
    Logger.recordOutput("$key/(3) XVelocity", getRelativeChassisSpeed().vxMetersPerSecond)
    Logger.recordOutput("$key/(4) YVelocity", getRelativeChassisSpeed().vyMetersPerSecond)
    Logger.recordOutput("$key/(5) Rotation2d", getHeadingRotation2d())
  }

  fun isAutoAlign() = autoAlign.get()

  fun toggleAutoAlign() = autoAlign.set(!autoAlign.get())

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * field.
   */
  fun drive(xSpeed: Double, ySpeed: Double, rotSpeed: Double, fieldRelative: Boolean, useConstDriveSpeed: Boolean, useConstRotSpeed: Boolean) {
    if (isX.get()) {
      setX()
      return
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    val xSpeedDelivered = xSpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useConstDriveSpeed) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.get())
    val ySpeedDelivered = ySpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useConstDriveSpeed) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.get())
    val rotDelivered = rotSpeed * SwerveConstants.MAX_ANGULAR_SPEED * (if (useConstRotSpeed) SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER else rotSpeedMultiplier.get())

    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      if (fieldRelative)
        ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered).apply {
          toRobotRelativeSpeeds(getHeadingRotation2d())
        }
      else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    )

    setDesiredModuleStates(*swerveModuleStates)
  }


  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  fun driveAutonomous(speeds: ChassisSpeeds) {
    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
    setDesiredModuleStates(*swerveModuleStates)
  }

  fun calculateSpeedForDesiredHeading(desiredHeading: Double): Double {
    headingController.setGoal(0.0)
    return headingController.calculate(desiredHeading)
  }

  fun setMaxSpeed(maxDrive: Double, maxRot: Double) {
    driveSpeedMultiplier.set(maxDrive)
    rotSpeedMultiplier.set(maxRot)
  }

  /**
   * Stops the robot.
   */
  fun stop() = drive(0.0, 0.0, 0.0, fieldRelative = false, useConstDriveSpeed = true, useConstRotSpeed = true)

  /**
   * It's important the SwerveModules are passed in with respect to the Kinematics construction.
   *
   * @return chassis speed relative to the robot.
   */
  fun getRelativeChassisSpeed(): ChassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
    *(io.getModules().map { it.getState() }.toTypedArray())
  )

  /**
   * @return the SwerveModulePositions of the SwerveModules.
   */
  fun getEstimatedPositions() = io.getModules().map { it.getPosition() }.toTypedArray()

  private fun getDesiredStates() = io.getModules().map { it.getDesiredState() }.toTypedArray()

  private fun getEstimatedStates() = io.getModules().map { it.getState() }.toTypedArray()

  /**
   * Sets the swerve ModuleStates. (FL, FR, BL, BR)
   *
   * @param desiredStates The desired SwerveModule states.
   */
  private fun setDesiredModuleStates(vararg desiredStates: SwerveModuleState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND)
    io.getModules().forEachIndexed { index, module -> module.setDesiredState(desiredStates[index]) }
  }

  private fun setX() {
    setDesiredModuleStates(
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
    )
  }

  fun toggleX() {
    isX.set(!isX.get())
  }

  fun isFieldRelative() = isFieldRelative.get()

  fun resetDriveEncoders() = io.getModules().forEach(Module::resetDriveEncoder)

  /**
   * Zeroes the gyro of the robot.
   */
  fun zeroGyro() = gyro.reset()

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  fun getHeadingRotation2d(): Rotation2d = Rotation2d.fromDegrees(getRawAngleDegrees())

  /**
   * The default SwerveMax template has an issue with inverting the Gyro, so the workaround is
   * manually negating the AHRS#getAngle. This function shouldn't get called by the user.
   *
   * @return The properly negated angle in degrees.
   */
  private fun getRawAngleDegrees() = (if (SwerveConstants.GYRO_REVERSED) -1.0 else 1.0) * gyro.getAngle()
}