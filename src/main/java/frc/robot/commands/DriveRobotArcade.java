// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveRobotArcade extends CommandBase {
    private DriveTrain m_driveTrain;
    private XboxController joystick;

  public DriveRobotArcade(RobotContainer robotContainer) {
    this.m_driveTrain = robotContainer.m_driveTrain;
    this.joystick = robotContainer.joystick;
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double move = this.joystick.getLeftY(); 
    double rot = this.joystick.getLeftX();
    this.m_driveTrain.arcadeDrive(-move, rot);
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
