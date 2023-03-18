// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private DriveTrain m_drive;

  private double m_setpoint;
  private double m_currentDistance;
  private double m_speed;

  private boolean m_backwards;

  public DriveDistance(DriveTrain drive, double distance, double speed, boolean backwards) {
    m_drive = drive;

    m_setpoint = distance;
    m_speed = speed;

    m_backwards = backwards;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(
        "========== Start DriveDistance( "
            + m_setpoint
            + " meters, "
            + m_backwards
            + " ) ==========");

    m_currentDistance = m_drive.getDistance();

    if (m_backwards) {
      m_setpoint = m_currentDistance - m_setpoint;
    } else {
      m_setpoint = m_currentDistance + m_setpoint;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentDistance = m_drive.getDistance();

    if (m_backwards) {
      m_drive.arcadeDrive(m_speed, 0, false);
    } else {
      m_drive.arcadeDrive(-m_speed, 0, false);
    }

    System.out.println("Drove: " + m_currentDistance + " meters");
    System.out.println("Distance to setpoint: " + (m_currentDistance - m_setpoint) + " meters");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("========== Finished DriveDistance() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_backwards) {
      return m_currentDistance < m_setpoint;
    } else {
      return m_currentDistance > m_setpoint;
    }
  }
}
