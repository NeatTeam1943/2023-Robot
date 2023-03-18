// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargeStationConstans;
import frc.robot.subsystems.DriveTrain;

public class DriveToChargeStaion extends CommandBase {

  private DriveTrain m_drive;

  private boolean m_backwards;
  private double m_angleY;

  public DriveToChargeStaion(DriveTrain driveTrain, boolean inverted) {
    m_drive = driveTrain;

    m_backwards = inverted;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== Start DriveToChargeStation( " + m_backwards + " ) ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angleY = m_drive.getGyroAngleY();

    if (m_backwards) {
      m_drive.arcadeDrive(ChargeStationConstans.kApproachSpeed, 0, false);
    } else {
      m_drive.arcadeDrive(-ChargeStationConstans.kApproachSpeed, 0, false);
    }

    System.out.println("Current angleY: " + m_angleY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
    System.out.println("========== Finished DriveToChargeStation() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_angleY < -ChargeStationConstans.kClimbAngleThreshold
        || m_angleY > ChargeStationConstans.kClimbAngleThreshold;
  }
}
