// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;

  public PhotonVision() {
    camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  public PhotonCamera GetCamera(){
    return camera;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets() != false) {
      PhotonTrackedTarget target = result.getBestTarget();
      int fiducialId = target.getFiducialId();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();

      // TODO: Save the fucking data somewhere (Use it)
    }
  }
}
