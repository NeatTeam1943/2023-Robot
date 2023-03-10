// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PassChargeStation extends CommandBase {

  private DriveTrain m_drive;
  private ADIS16448_IMU m_imu;

  private boolean m_onChargingStation = false;
  private boolean m_passChargingStation = false;
  private boolean m_hasSafeDistance = false;

  private boolean m_passedStaition = false;

  private boolean m_backwards;

  private double m_setpoint;
  final double voltage = 0.2;
  final double downVoltage = 0.15;

  private final double m_distance = 0.25;

  public PassChargeStation(DriveTrain driveTrain, boolean backwards) {
    m_drive = driveTrain;
    m_imu = m_drive.getIMU();
    m_backwards = backwards;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_imu.reset();
    m_imu.calibrate();
    m_onChargingStation = false;
    m_passChargingStation = false;
    m_hasSafeDistance = false;
    m_passedStaition = false;

    System.out.println("========== Start PassChargeStation( " + m_backwards + " ) ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_backwards) {
      if (m_imu.getGyroAngleY() > 10) {
        m_onChargingStation = true;
      }

      if (m_imu.getGyroAngleY() > -1 && m_imu.getGyroAngleY() < 1 && m_onChargingStation) {
        m_passChargingStation = true;
      }

      if (m_onChargingStation && m_passChargingStation && !m_passedStaition) {
        m_setpoint = m_drive.getDistance() - m_distance;
        m_passedStaition = true;
      }

      if (m_drive.getDistance() < m_setpoint && m_passedStaition) {
        m_hasSafeDistance = true;
      }

    } else {
      if (m_imu.getGyroAngleY() > 10) {
        m_onChargingStation = true;
      }

      if (m_imu.getGyroAngleY() > -1 && m_imu.getGyroAngleY() < 1 && m_onChargingStation) {
        m_passChargingStation = true;
      }

      if (m_onChargingStation && m_passChargingStation && !m_passedStaition) {
        m_setpoint = m_drive.getDistance() + m_distance;
        m_passedStaition = true;
      }

      if (m_drive.getDistance() > m_setpoint && m_passedStaition) {
        m_hasSafeDistance = true;
      }
    }

    if (m_backwards) {
      if (!m_onChargingStation) {
        m_drive.arcadeDrive(voltage, 0, false);
      } else {
        m_drive.arcadeDrive(downVoltage, 0, false);
      }
    } else {
      if (!m_onChargingStation) {
        m_drive.arcadeDrive(-voltage, 0, false);
      } else {
        m_drive.arcadeDrive(-downVoltage, 0, false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
    System.out.println("========== Finished PassChargeStation() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_onChargingStation && m_passChargingStation && m_hasSafeDistance;
  }
}
