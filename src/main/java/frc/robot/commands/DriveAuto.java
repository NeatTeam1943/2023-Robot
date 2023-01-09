// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveAuto extends CommandBase {
  /** Creates a new DriveAuto. */
  public DriveAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveArcade.getBackLeftMotor().setSelectedSensorPosition(Constants.kResetEncoder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drivingDistance = RobotContainer.driveArcade.getBackLeftMotor().getSelectedSensorPosition();
    if (drivingDistance < 5) 
      RobotContainer.driveArcade.robotArcadeDrive(Constants.kAutoSpeed, 0);
    else
      RobotContainer.driveArcade.robotArcadeDrive(0, 0);

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
