// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

public class EncoderAuto extends CommandBase {
  /** Creates a new EncoderAuto. */
  private DriveTrain m_driveTrain;

  private double m_distance;

  public EncoderAuto(DriveTrain driveTrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_distance = distance;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(AutoConstants.kSpeed, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0, false);
    m_driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getDistance() / 100 > m_distance;
  }
}
