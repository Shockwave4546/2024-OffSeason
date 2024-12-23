package org.shockwave.subsystem.swervenew

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.shockwave.GlobalConstants
import org.shockwave.utils.LocalADStarAK
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.withSign

class SwerveSubsystem(private val gyroIO: GyroIO, flModuleIO: ModuleIO, frModuleIO: ModuleIO, blModuleIO: ModuleIO, brModuleIO: ModuleIO) : SubsystemBase() {
  private val gyroInputs = GyroIO.GyroIOInputs()
  private val modules = arrayOf(
    Module(flModuleIO, ModulePosition.FRONT_LEFT),
    Module(frModuleIO, ModulePosition.FRONT_RIGHT),
    Module(blModuleIO, ModulePosition.BACK_LEFT),
    Module(brModuleIO, ModulePosition.BACK_RIGHT)
  )
  private var rawGyroRotation = Rotation2d()
//  private val sysId: SysIdRoutine = null

  private val lastModulePositions =
    arrayOf(
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition()
    )
  private val poseEstimator = SwerveDrivePoseEstimator(SwerveConstants.DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, Pose2d())

  init {
    SparkOdometryThread.start()

    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getChassisSpeeds,
      this::runVelocity,
      PPHolonomicDriveController(SwerveConstants.AUTO_DRIVE_PID, SwerveConstants.AUTO_TURN_PID),
      SwerveConstants.PATH_PLANNER_ROBOT_CONFIG,
      { DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red },
      this
    )

    Pathfinding.setPathfinder(LocalADStarAK())
    PathPlannerLogging.setLogActivePathCallback { activePath ->
      Logger.recordOutput("Odometry/Trajectory", *activePath.toTypedArray<Pose2d>())
    }
    PathPlannerLogging.setLogTargetPoseCallback { targetPose ->
      Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
    }
  }

  override fun periodic() {
    ODOMETRY_LOCK.lock() // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs)
    Logger.processInputs("Drive/Gyro", gyroInputs)
    modules.forEach(Module::periodic)
    ODOMETRY_LOCK.unlock()

    // Stop moving when disabled & Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      modules.forEach(Module::stop)
      Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
      Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
    }

    // Update odometry
    val sampleTimestamps = modules[0].getOdometryTimestamps() // All signals are sampled together
    val sampleCount = sampleTimestamps.size
    for (i in 0..<sampleCount) {
      // Read wheel positions and deltas from each module
      val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
      val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
      for (moduleIndex in 0..3) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i]
        moduleDeltas[moduleIndex] =
          SwerveModulePosition(
            modulePositions[moduleIndex]!!.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex]!!.angle
          )
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex]!!
      }

      rawGyroRotation = gyroInputs.odometryYawPositions[i]
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions)
    }
  }

  fun runVelocity(speeds: ChassisSpeeds) {
    // Calculate module setpoints
    speeds.discretize(0.02)
    val setpointStates: Array<SwerveModuleState> = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND)

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds)

    // Send setpoints to modules
    for (i in 0..3) {
      modules[i].runSetpoint(setpointStates[i])
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  fun stopWithX() {
    val headings = arrayOfNulls<Rotation2d>(4)
    for (i in 0..3) {
      headings[i] = SwerveConstants.MODULE_TRANSLATIONS[i].angle
    }
    SwerveConstants.DRIVE_KINEMATICS.resetHeadings(*headings)
    stop()
  }

  fun stop() = runVelocity(ChassisSpeeds())

  /** Returns the module states (turn angles and drive velocities) for all of the modules.  */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private fun getModuleStates(): Array<SwerveModuleState?> {
    val states = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0..3) {
      states[i] = modules[i].getState()
    }

    Logger.recordOutput("SwerveStates/Measured", *states)
    return states
  }

  private fun getModulePositions(): Array<SwerveModulePosition?> {
    val states = arrayOfNulls<SwerveModulePosition>(4)
    for (i in 0..3) {
      states[i] = modules[i].getPosition()
    }
    return states
  }

  /** Returns the measured chassis speeds of the robot.  */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private fun getChassisSpeeds() = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates())

  @AutoLogOutput(key = "Odometry/Robot")
  fun getPose() = poseEstimator.estimatedPosition

  fun getRotation() = getPose().rotation

  fun setPose(pose: Pose2d) = poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose)

  fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
  }

  companion object {
    val ODOMETRY_LOCK = ReentrantLock()

    private fun getLinearVelocityFromJoysticks(x: Double, y: Double): Translation2d {
      // Apply deadband
      var linearMagnitude = MathUtil.applyDeadband(hypot(x, y), GlobalConstants.DRIVE_DEADBAND)
      val linearDirection = Rotation2d(atan2(y, x))

      // Square magnitude for more precise control
      linearMagnitude *= linearMagnitude

      // Return new linear velocity
      return Pose2d(Translation2d(), linearDirection)
        .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
        .translation
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    fun joystickDrive(swerve: SwerveSubsystem, xSupplier: DoubleSupplier, ySupplier: DoubleSupplier, omegaSupplier: DoubleSupplier): Command = Commands.run(
      {
        val linearVelocity = getLinearVelocityFromJoysticks(xSupplier.asDouble, ySupplier.asDouble)

        // Apply rotation deadband
        var omega = MathUtil.applyDeadband(omegaSupplier.asDouble, GlobalConstants.DRIVE_DEADBAND)

        // Square rotation value for more precise control
        omega = (omega * omega).withSign(omega)

        // Convert to field relative speeds & send command
        val speeds =
          ChassisSpeeds(
            linearVelocity.x * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            linearVelocity.y * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            omega * SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SECOND
          )
        val isFlipped = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        speeds.toRobotRelativeSpeeds(
          if (isFlipped) swerve.getRotation().plus(Rotation2d(Math.PI)) else swerve.getRotation()
        )
        swerve.runVelocity(speeds)
      },
      swerve
    )
  }
}