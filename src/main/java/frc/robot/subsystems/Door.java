// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoorConstants;

public class Door extends SubsystemBase {
  private DigitalInput m_switchOpen;
  private DigitalInput m_switchClose;

  private WPI_TalonSRX m_motor;

  public Door() {
    m_switchOpen = new DigitalInput(DoorConstants.kOpenSwitch);
    m_switchClose = new DigitalInput(DoorConstants.kCloseSwitch);

    m_motor = new WPI_TalonSRX(DoorConstants.kDoorMotorID);
  }

  public boolean isOpen() {
    return m_switchOpen.get();
  }

  public boolean isClosed() {
    return m_switchClose.get();
  }

  public void move(double value) {
    m_motor.set(value);
  }

  @Override
  public void periodic() {
  }
}
