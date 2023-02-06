// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveRobotArcade extends CommandBase {
  /** Creates a new DriveRobotArcade. */
  public DriveRobotArcade() {
    addRequirements(RobotContainer.getDriveTrain());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double move = RobotContainer.getJoystick().getLeftY(); 
    double rot = RobotContainer.getJoystick().getLeftX();
    RobotContainer.getDriveTrain().arcadeDrive(-move, rot);
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
