// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  DriveTrain m_drive;

  private double m_meters;
  private double m_setpoint;
  private boolean m_inverted;
  private final double m_voltage = 0.25;
  /** Creates a new driveMeters. */
  public DriveDistance(DriveTrain drive, double meters, boolean inverted) {
    m_drive = drive;
    m_meters = meters;
    m_inverted = inverted;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_inverted){
      m_setpoint = m_drive.getDistance() - m_meters;
    } else {
      m_setpoint = m_drive.getDistance() + m_meters;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_inverted){
      m_drive.arcadeDrive(m_voltage, 0, false);
    } else {
      m_drive.arcadeDrive(-m_voltage, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_inverted){
      return m_drive.getDistance() < m_setpoint;
    } else {
      return m_drive.getDistance() > m_setpoint;
    }
  }
}
