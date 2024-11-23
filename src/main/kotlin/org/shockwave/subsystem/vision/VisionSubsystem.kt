package org.shockwave.subsystem.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonPoseEstimator
import org.shockwave.RobotContainer
import org.shockwave.shuffleboard.ShuffleboardBoolean
import org.shockwave.subsystem.swerve.SwerveConstants.Companion.DRIVE_KINEMATICS
import org.shockwave.subsystem.swerve.SwerveSubsystem
import org.shockwave.subsystem.vision.VisionConstants.Companion.FRONT_CAMERA_NAME
import org.shockwave.subsystem.vision.VisionConstants.Companion.STATE_STD_DEVS
import org.shockwave.subsystem.vision.VisionConstants.Companion.VISION_MEASUREMENT_STD_DEVS
import org.shockwave.utils.Tab
import org.shockwave.utils.TunableBoolean
import java.util.*
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class VisionSubsystem(private val vision: VisionIO, private val swerve: SwerveSubsystem) : SubsystemBase() {
  private val layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
  private val photonPoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
    layout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    VisionConstants.FRONT_CAMERA_TO_ROBOT
  )
  private val swervePoseEstimator: SwerveDrivePoseEstimator
  private val key = "Vision"
  private val useVisionMeasurement = TunableBoolean("$key/Use Vision Measurement", false)
  private val canShoot = ShuffleboardBoolean(Tab.MATCH, "Can Shoot", false).withSize(3, 3)

  init {
    photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)

    swervePoseEstimator = SwerveDrivePoseEstimator(
      DRIVE_KINEMATICS,
      Rotation2d(),
      arrayOf(
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
      ),
      Pose2d(),
      STATE_STD_DEVS,
      VISION_MEASUREMENT_STD_DEVS
    )
  }

  override fun periodic() {
    swervePoseEstimator.update(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions())
    if (!useVisionMeasurement.get()) return
    vision.getUnreadResults().forEach { result ->
      photonPoseEstimator.update(result).ifPresent { estimatedPose ->
        val pose = estimatedPose.estimatedPose
        if (!isValidMeasurement(pose, false)) return@ifPresent
        swervePoseEstimator.addVisionMeasurement(pose.toPose2d(), result.timestampSeconds)
      }
    }

    val toSpeaker = getToSpeaker()
    val canShoot = toSpeaker.distance.isPresent && toSpeaker.distance.get() < VisionConstants.SHOOTABLE_DISTANCE
    this.canShoot.set(canShoot)

    Logger.recordOutput("$key/(1) EstimatedPose2d", getPose2d())

    Logger.recordOutput("$key/(2) IsSpeakerVisible", toSpeaker.distance.isPresent)
    Logger.recordOutput("$key/(3) SpeakerDistance", toSpeaker.distance.getOrDefault(-1.0))
    Logger.recordOutput("$key/(4) SpeakerRotation", toSpeaker.rotation2d.getOrDefault(Rotation2d()))
    Logger.recordOutput(
      "$key/(5) SpeakerRotDegrees",
      toSpeaker.rotation2d.getOrDefault(Rotation2d()).degrees
    )

    Logger.recordOutput("$key/(6) IsHeadingTowardSpeaker", isHeadingAlignedWithSpeaker().getOrDefault(false))
  }

  fun getDistanceToTags() = buildMap {
    for (result in vision.getUnreadResults()) {
      for (target in result.targets) {
        val camToTarget = target.bestCameraToTarget
        val robotToTarget = VisionConstants.ROBOT_TO_FRONT_CAMERA.plus(camToTarget)
        val calcDistance = sqrt(robotToTarget.x.pow(2.0) + robotToTarget.y.pow(2.0))
        put(target.fiducialId, calcDistance)
      }
    }
  }

  fun getAngleToTags() = buildMap {
    for (result in vision.getUnreadResults()) {
      for (target in result.targets) {
        put(target.fiducialId, Rotation2d.fromDegrees(target.yaw))
      }
    }
  }

  fun getDistanceToTag(id: Int): Optional<Double> {
    var distance = Optional.empty<Double>()
    for (result in vision.getUnreadResults()) {
      for (target in result.targets) {
        if (target.fiducialId != id) continue
        val camToTarget = target.bestCameraToTarget
        val robotToTarget = VisionConstants.ROBOT_TO_FRONT_CAMERA.plus(camToTarget)

        val calcDistance = sqrt(robotToTarget.x.pow(2.0) + robotToTarget.y.pow(2.0))
        distance = Optional.of(calcDistance)
        break
      }
    }

    return distance
  }

  fun getRotation2dToTag(id: Int): Optional<Rotation2d> {
    var angle = Optional.empty<Rotation2d>()
    for (result in vision.getUnreadResults()) {
      for (target in result.targets) {
        if (target.fiducialId != id) continue
        angle = Optional.of(Rotation2d.fromDegrees(target.yaw))
        break
      }
    }

    return angle
  }

  fun getToSpeaker() = DistanceAnglePair(
    getDistanceToTag(getSpeakerTargetTagId()),
    getRotation2dToTag(getSpeakerTargetTagId())
  )

  fun isHeadingAlignedWithSpeaker(): Optional<Boolean> {
    val speakerRot = getRotation2dToTag(getSpeakerTargetTagId())
    return if (speakerRot.isPresent) Optional.of(abs(speakerRot.get().radians) < VisionConstants.HEADING_TOLERANCE) else Optional.empty()
  }

  private fun getSpeakerTagPose3d() = layout.getTagPose(getSpeakerTargetTagId()).get()

  private fun isValidMeasurement(pose: Pose3d, printError: Boolean) = when {
    pose.x > VisionConstants.FIELD_LENGTH -> {
      if (printError) DriverStation.reportError(
        "According to $FRONT_CAMERA_NAME, Robot is off the field in +x direction.",
        false
      )
      false
    }

    pose.x < 0 -> {
      if (printError) DriverStation.reportError(
        "According to ${FRONT_CAMERA_NAME}, Robot is off the field in -x direction.",
        false
      )
      false
    }

    pose.y > VisionConstants.FIELD_WIDTH -> {
      if (printError) DriverStation.reportError(
        "According to ${FRONT_CAMERA_NAME}, Robot is off the field in +y direction.",
        false
      )
      false
    }

    pose.y < 0 -> {
      if (printError) DriverStation.reportError(
        "According to ${FRONT_CAMERA_NAME}, Robot is off the field in -y direction.",
        false
      )
      false
    }

    pose.z < -0.15 -> {
      if (printError) DriverStation.reportError(
        "According to $FRONT_CAMERA_NAME, Robot is inside the floor.",
        false
      )
      false
    }

    pose.z > 0.15 -> {
      if (printError) DriverStation.reportError(
        "According to $FRONT_CAMERA_NAME, Robot is floating above the floor.",
        false
      )
      false
    }

    else -> true
  }

  fun getPose2d(): Pose2d = swervePoseEstimator.estimatedPosition

  fun resetPose(pose: Pose2d) =
    swervePoseEstimator.resetPosition(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions(), pose)

  fun resetFieldOrientatedPose() = swervePoseEstimator.resetPosition(
    swerve.getHeadingRotation2d(),
    swerve.getEstimatedPositions(),
    Pose2d(getPose2d().translation, Rotation2d())
  )

  companion object {
    fun getSpeakerTargetTagId() =
      if (RobotContainer.isRedAlliance()) VisionConstants.RED_SPEAKER_ID else VisionConstants.BLUE_SPEAKER_ID
  }
}