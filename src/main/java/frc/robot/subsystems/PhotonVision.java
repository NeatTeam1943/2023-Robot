// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonTrackedTarget target;
  private int fiducialId;
  private Transform3d bestCameraToTarget;
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  private Transform3d pose;
  private List<TargetCorner> corners;
  private double poseAmbiguity;
  private Transform3d alternateCameraToTarget;

  public PhotonVision() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
  }

  public PhotonCamera getCamera(){
    return m_camera;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets() != false) {
      target = result.getBestTarget();
      fiducialId = target.getFiducialId();
      bestCameraToTarget = target.getBestCameraToTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
      corners = target.getDetectedCorners();
    }
  }
}
