// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SimDrive;

public class AutoDrive extends CommandBase {
  SimDrive m_driveTrain;
  Timer m_timer;
  /** Creates a new autoDrive. */
  public AutoDrive(SimDrive driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain; 
    m_timer = new Timer();  
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() < 5)
      m_driveTrain.robotArcadeDrive(0, 1);
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
