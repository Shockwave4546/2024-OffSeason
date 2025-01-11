package org.shockwave.subsystem.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
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
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.GlobalConstants
import org.shockwave.utils.LocalADStarAK
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.*
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier
import kotlin.math.abs
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

    Logger.recordOutput("SwerveStates/Measured", *getModuleStates())
    Logger.recordOutput("Odometry/Robot", getPose())
    Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds())

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
    ChassisSpeeds.discretize(speeds, 0.2)
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

  fun runCharacterization(voltage: Double) = modules.forEach { it.runCharacterization(voltage) }

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
  private fun getModuleStates() = modules.map { it.getState() }.toTypedArray()

  private fun getModulePositions() = modules.map { it.getPosition() }.toTypedArray()

  private fun getWheelRadiusCharacterizationPositions() = modules.map { it.getWheelRadiusCharacterizationPosition() }.toDoubleArray()

  private fun getFFCharacterizationVelocity() = modules.sumOf { it.getFFCharacterizationVelocity() }.div(4)

  /** Returns the measured chassis speeds of the robot.  */
  private fun getChassisSpeeds() = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(*getModuleStates())

  fun getPose() = poseEstimator.estimatedPosition

  fun getRotation() = getPose().rotation

  fun setPose(pose: Pose2d) = poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose)

  fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
  }

  companion object {
    val ODOMETRY_LOCK = ReentrantLock()

    private const val WHEEL_RADIUS_MAX_VELOCITY: Double = 0.25 // Rad/Sec
    private const val WHEEL_RADIUS_RAMP_RATE: Double = 0.05 // Rad/Sec^2
    private class WheelRadiusCharacterizationState {
      var positions = DoubleArray(4)
      var lastAngle = Rotation2d()
      var gyroDelta = 0.0
    }

    fun wheelRadiusCharacterization(swerve: SwerveSubsystem): Command {
      val limiter = SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE)
      val state = WheelRadiusCharacterizationState()

      return Commands.parallel( // Drive control sequence
        Commands.sequence( // Reset acceleration limiter
          Commands.runOnce(
            {
              limiter.reset(0.0)
            }),  // Turn in place, accelerating up to full speed

          Commands.run(
            {
              val speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)
              swerve.runVelocity(ChassisSpeeds(0.0, 0.0, speed))
            },
            swerve
          )
        ),  // Measurement sequence

        Commands.sequence( // Wait for modules to fully orient before starting measurement
          Commands.waitSeconds(1.0),  // Record starting measurement

          Commands.runOnce(
            {
              state.positions = swerve.getWheelRadiusCharacterizationPositions()
              state.lastAngle = swerve.getRotation()
              state.gyroDelta = 0.0
            }),  // Update gyro delta

          Commands.run(
            {
              val rotation = swerve.getRotation()
              state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
              state.lastAngle = rotation
            }) // When cancelled, calculate and print results

            .finallyDo(
              Runnable {
                val positions = swerve.getWheelRadiusCharacterizationPositions()
                var wheelDelta = 0.0
                for (i in 0..3) {
                  wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                }
                val wheelRadius: Double =
                  (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta

                val formatter: NumberFormat = DecimalFormat("#0.000")
                println(
                  "********** Wheel Radius Characterization Results **********"
                )
                println(
                  "\tWheel Delta: " + formatter.format(wheelDelta) + " radians"
                )
                println(
                  "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians"
                )
                println(
                  ("\tWheel Radius: "
                          + formatter.format(wheelRadius)
                          + " meters, "
                          + formatter.format(Units.metersToInches(wheelRadius))
                          + " inches")
                )
              })
        )
      )
    }

    private const val FF_START_DELAY = 2.0 // Secs
    private const val FF_RAMP_RATE: Double = 0.1 // Volts/Sec
    fun feedforwardCharacterization(swerve: SwerveSubsystem): Command {
      val velocitySamples = LinkedList<Double>()
      val voltageSamples = LinkedList<Double>()
      val timer = Timer()

      return Commands.sequence( // Reset data
        Commands.runOnce(
          {
            velocitySamples.clear()
            voltageSamples.clear()
          }),  // Allow modules to orient

        Commands.run(
          {
            swerve.runCharacterization(0.0)
          },
          swerve
        )
          .withTimeout(FF_START_DELAY),  // Start timer

        Commands.runOnce({ timer.restart() }),  // Accelerate and gather data

        Commands.run(
          {
            val voltage: Double = timer.get() * FF_RAMP_RATE
            swerve.runCharacterization(voltage)
            velocitySamples.add(swerve.getFFCharacterizationVelocity())
            voltageSamples.add(voltage)
          },
          swerve
        ) // When cancelled, calculate and print results

          .finallyDo(
            Runnable {
              val n = velocitySamples.size
              var sumX = 0.0
              var sumY = 0.0
              var sumXY = 0.0
              var sumX2 = 0.0
              for (i in 0..<n) {
                sumX += velocitySamples[i]
                sumY += voltageSamples[i]
                sumXY += velocitySamples[i] * voltageSamples[i]
                sumX2 += velocitySamples[i] * velocitySamples[i]
              }
              val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
              val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

              val formatter: NumberFormat = DecimalFormat("#0.00000")
              println("********** Drive FF Characterization Results **********")
              println("\tkS: " + formatter.format(kS))
              println("\tkV: " + formatter.format(kV))
            })
      )
    }

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
        swerve.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            if (isFlipped) swerve.getRotation().plus(Rotation2d(Math.PI)) else swerve.getRotation()
          )
        )
        swerve.runVelocity(speeds)
      },
      swerve
    )
  }
}