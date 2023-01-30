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
import org.photonvision.targeting.TargetCorner;

public class PhotonVision extends SubsystemBase {

  private static PhotonCamera camera;

  public PhotonVision() {
    camera = new PhotonCamera(VisionConstants.kCameraName);

  }
  public static PhotonCamera getCamera(){
    return camera;

  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    if(result.hasTargets() != false){
      var target = result.getBestTarget();
      var fiducialId = target.getFiducialId();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();

    }
    // This method will be called once per scheduler run
  }
}
