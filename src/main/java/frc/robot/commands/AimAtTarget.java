// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class AimAtTarget extends CommandBase {
  private PhotonVision m_photonVision;
  private DriveTrain m_driveTrain;
  private PhotonCamera m_camera;
  private PIDController m_forwardController;
  private PIDController m_turnController;
  private PhotonPipelineResult result;

  private double m_forwardSpeed;
  private double m_angularSpeed;
  private double m_distance;
  private double m_targetHeight;
  private double m_targetPitch;

  /** Creates a new AimAtTarget. */
  public AimAtTarget(PhotonCamera camera) {
    m_photonVision = new PhotonVision();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_photonVision, m_driveTrain);

    m_camera = camera;

    m_forwardController = new PIDController(VisionConstants.kForwardP, VisionConstants.kForwardI, VisionConstants.kForwardD);
    m_turnController = new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI, VisionConstants.kTurnD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_camera = m_photonVision.getCamera();
    m_targetPitch = Units.degreesToRadians(result.getBestTarget().getPitch());

    if (m_camera.getPipelineIndex() == 0) {
      m_targetHeight = VisionConstants.kAprilTagHeight;
    } else {
      m_targetHeight = VisionConstants.kRetroReflectiveHeight;
    }

    m_distance = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kCameraHeight,
                                                             m_targetHeight,
                                                             VisionConstants.kCameraPitchRadians,
                                                             m_targetPitch);

    m_forwardSpeed = m_forwardController.calculate(m_distance);
    m_angularSpeed = m_turnController.calculate(result.getBestTarget().getYaw());

    m_driveTrain.arcadeDrive(m_forwardSpeed, m_angularSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
