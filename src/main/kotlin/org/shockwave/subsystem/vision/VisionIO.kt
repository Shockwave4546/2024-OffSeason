package org.shockwave.subsystem.vision

import org.photonvision.targeting.PhotonPipelineResult

interface VisionIO {
  fun getUnreadResults(): MutableList<PhotonPipelineResult>
}