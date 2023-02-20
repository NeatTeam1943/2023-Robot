// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVision;

public class TogglePipeline extends CommandBase {
  private final PhotonVision m_photonVision;
  private final int m_pipelineIndex;

  public TogglePipeline(PhotonVision photon, int pipe) {
    m_photonVision = photon;
    m_pipelineIndex = pipe;

    addRequirements(m_photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_photonVision.getCamera().setPipelineIndex(m_pipelineIndex);
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
