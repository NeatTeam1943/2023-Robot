// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.PrinterURI;
import javax.swing.plaf.basic.BasicLookAndFeel;
import javax.xml.crypto.KeySelector.Purpose;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GyroAuto extends CommandBase {

  private DriveTrain m_drive;
  private Timer m_time;

  /** Creates a new GyroAuto. */
  public GyroAuto(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveTrain;
    m_time = new Timer();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getIMU().calibrate();
    m_time.reset();
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(-0.15, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_time.stop();
    m_drive.getIMU().calibrate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_time.get() > 2 && m_drive.getIMU().getGyroAngleX() > -1 && m_drive.getIMU().getGyroAngleX() < 1;
  }
}
