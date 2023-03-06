// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToCommunity extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  private boolean m_onChargingStation = false;
  private boolean m_passChargingStation = false;
  private boolean m_hasSafeDistance = false;
  
  // Arabic line
  private boolean m_passedStaition = false; 

  private boolean m_inverted;

  private double m_setpoint;

  /** Creates a new GyroAuto. */
  public DriveToCommunity(DriveTrain driveTrain, boolean inverted) {
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();
    m_inverted = inverted;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_imu.calibrate();
    m_onChargingStation = false;
    m_passChargingStation = false;
    m_hasSafeDistance = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double voltage = 0.15;
    if (m_inverted){
      if (m_imu.getGyroAngleX() < -10) {
        m_onChargingStation = true;
      }
  
      if (m_imu.getGyroAngleX() > -1 && m_imu.getGyroAngleX() < 1 && m_onChargingStation) {
        m_passChargingStation = true; 
      }
  
      if (m_onChargingStation && m_passChargingStation && !m_passedStaition){
        m_setpoint = m_drive.getDistance() - 0.25; 
        m_passedStaition = true; 
      }
  
      if (m_drive.getDistance() < m_setpoint && m_passedStaition){
        m_hasSafeDistance = true; 
      }
  
  
    } else{
      if (m_imu.getGyroAngleX() > 10) {
        m_onChargingStation = true;
      }
  
      if (m_imu.getGyroAngleX() > -1 && m_imu.getGyroAngleX() < 1 && m_onChargingStation) {
        m_passChargingStation = true; 
      }
  
      if (m_onChargingStation && m_passChargingStation && !m_passedStaition){
        m_setpoint = m_drive.getDistance() + 0.25; 
        m_passedStaition = true; 
      }
  
      if (m_drive.getDistance() > m_setpoint && m_passedStaition){
        m_hasSafeDistance = true; 
      }
    }

    if (m_inverted){
      m_drive.arcadeDrive(voltage, 0, false);
    } else{
      m_drive.arcadeDrive(-voltage, 0, false);
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
    return m_onChargingStation && m_passChargingStation && m_hasSafeDistance;
  }
}
