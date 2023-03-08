// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Climb extends CommandBase {

  private DriveTrain m_drive;

  private double m_setpoint;
  private boolean m_inverted;

  private double m_distance = 1.1;

  /** Creates a new GyroAuto. */
  public Climb(DriveTrain driveTrain, boolean invert) {
    m_drive = driveTrain;
    m_inverted = invert; 

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_inverted){
      m_setpoint = m_drive.getDistance() - m_distance;
    } else {
      m_setpoint = m_drive.getDistance() + m_distance; 
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("climb");
    System.out.println("difference: "+(m_drive.getDistance() - m_setpoint));
    final double voltage = -0.15;

    if (m_inverted) {
      m_drive.arcadeDrive(-voltage, 0, false);
    } else {
      m_drive.arcadeDrive(voltage, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_inverted) {
      return m_drive.getDistance() < m_setpoint;
    } else {
      return m_drive.getDistance() > m_setpoint;
    }
  }
}