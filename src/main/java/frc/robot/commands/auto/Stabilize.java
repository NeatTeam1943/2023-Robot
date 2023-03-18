// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StabilizeConstants;
import frc.robot.subsystems.DriveTrain;

public class Stabilize extends CommandBase {

  private DriveTrain m_drive;

  public Stabilize(DriveTrain driveTrain) {
    m_drive = driveTrain;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== Start Stabilize() ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double angleY = m_drive.getGyroAngleY();

    if (angleY > StabilizeConstants.kAngleThreshold) {
      m_drive.arcadeDrive(-StabilizeConstants.kSlowSpeed, 0, false);
      System.out.println("Going forward, angleY: " + angleY);
    } else if (angleY < -StabilizeConstants.kAngleThreshold) {
      m_drive.arcadeDrive(StabilizeConstants.kSlowSpeed, 0, false);
      System.out.println("Going backwards, angleY: " + angleY);
    } else {
      m_drive.arcadeDrive(0, 0, false);
      System.out.println("Stable, angleY: " + angleY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
    System.out.println("========== Finished Stabilize() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
