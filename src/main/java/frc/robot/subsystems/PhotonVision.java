// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonTrackedTarget target;

  public PhotonVision() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  public PhotonCamera getCamera(){
    return m_camera;
  }

  public double getDistance(double targetHeight, double targetPitch){
    double distance = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kCameraHeight,
    targetHeight,
    VisionConstants.kCameraPitchRadians,
    targetPitch);

    return distance;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets() != false) {
      target = result.getBestTarget();
    }
  }
}

