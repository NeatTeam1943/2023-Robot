// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;

public class TogglePipeline extends CommandBase {
  private final PhotonVision m_photonVision;
  private final Intake m_intake;

  public TogglePipeline(PhotonVision photon, Intake intake) {
    m_photonVision = photon;
    m_intake = intake;

    addRequirements(m_photonVision,m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_photonVision.getCamera().setPipelineIndex(m_intake.getDetectedGamePiece());
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
