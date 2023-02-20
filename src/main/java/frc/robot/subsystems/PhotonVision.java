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
  private PhotonTrackedTarget m_target;

  public PhotonVision() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  public PhotonCamera getCamera() {
    return m_camera;
  }

  public double getDistance(double targetHeight, double targetPitch) {
    double m_distance = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kCameraHeight,
    targetHeight,
    VisionConstants.kCameraPitchRadians,
    targetPitch);

    return m_distance;
  }

  public PhotonTrackedTarget getM_target(){
    return m_target;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult m_result = m_camera.getLatestResult();
    if (m_result.hasTargets() != false) {
      m_target = m_result.getBestTarget();
    }
  }
}

