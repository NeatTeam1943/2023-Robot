// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargeStationConstans;
import frc.robot.Constants.PassChargeStationConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class PassChargeStation extends CommandBase {

  private DriveTrain m_drive;

  private boolean m_onChargingStation = false;
  private boolean m_passChargingStation = false;
  private boolean m_hasSafeDistance = false;
  private boolean m_passedStaition = false;

  private boolean m_backwards;

  private double m_setpoint;
  private double m_unsafeSetPoint;

  public PassChargeStation(DriveTrain driveTrain, boolean backwards) {
    m_drive = driveTrain;
    m_backwards = backwards;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.calibrateIMU();
    m_onChargingStation = false;
    m_passChargingStation = false;
    m_hasSafeDistance = false;
    m_passedStaition = false;

    if (m_backwards) {
      m_unsafeSetPoint = m_drive.getDistance() - PassChargeStationConstants.kUnsafeDistance;
    } else {
      m_unsafeSetPoint = m_drive.getDistance() + PassChargeStationConstants.kUnsafeDistance;
    }

    System.out.println("========== Start PassChargeStation( " + m_backwards + " ) ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double angleY = m_drive.getGyroAngleY();
    final double currentDistance = m_drive.getDistance();

    System.out.println("angle: " + angleY);

    if (m_backwards) {
      if (angleY == 0 && currentDistance < m_unsafeSetPoint) {
        Robot.cancelAuto();
      }

      if (angleY > ChargeStationConstans.kClimbAngleThreshold) {
        m_onChargingStation = true;
      }

      if (m_drive.isStable() && m_onChargingStation) {
        m_passChargingStation = true;
      }

      if (m_onChargingStation && m_passChargingStation && !m_passedStaition) {
        m_setpoint = currentDistance - PassChargeStationConstants.kSafeDistance;
        m_passedStaition = true;
      }

      if (currentDistance < m_setpoint && m_passedStaition) {
        m_hasSafeDistance = true;
      }

    } else {
      if (angleY == 0 && currentDistance > m_unsafeSetPoint) {
        System.out.println("IMU FAILURE! CANCELLING AUTONOMOUS!");
        Robot.cancelAuto();
      }

      if (angleY < -ChargeStationConstans.kClimbAngleThreshold) {
        m_onChargingStation = true;
      }

      if (m_drive.isStable() && m_onChargingStation) {
        m_passChargingStation = true;
      }

      if (m_onChargingStation && m_passChargingStation && !m_passedStaition) {
        m_setpoint = currentDistance + PassChargeStationConstants.kSafeDistance;
        m_passedStaition = true;
      }

      if (currentDistance > m_setpoint && m_passedStaition) {
        m_hasSafeDistance = true;
      }
    }

    if (m_backwards) {
      if (!m_onChargingStation) {
        m_drive.arcadeDrive(PassChargeStationConstants.kUpSpeed, 0, false);
      } else {
        m_drive.arcadeDrive(PassChargeStationConstants.kDownSpeed, 0, false);
      }
    } else {
      if (!m_onChargingStation) {
        m_drive.arcadeDrive(-PassChargeStationConstants.kUpSpeed, 0, false);
      } else {
        m_drive.arcadeDrive(-PassChargeStationConstants.kDownSpeed, 0, false);
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
