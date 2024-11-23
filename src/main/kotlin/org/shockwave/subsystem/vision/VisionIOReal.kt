package org.shockwave.subsystem.vision

import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult

class VisionIOReal : VisionIO {
  private val frontCamera = PhotonCamera(VisionConstants.FRONT_CAMERA_NAME)

  override fun getUnreadResults(): MutableList<PhotonPipelineResult> = frontCamera.allUnreadResults
}