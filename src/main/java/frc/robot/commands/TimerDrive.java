// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

public class TimerDrive extends CommandBase {
  /** Creates a new TimerDrive. */
  private Timer m_time;
  private DriveTrain m_driveTrain;

  public TimerDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_time = new Timer();

    m_driveTrain = driveTrain; 

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.reset();
    m_time.start();
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
    m_time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_time.get() > AutoConstants.kTime;
  }
}
