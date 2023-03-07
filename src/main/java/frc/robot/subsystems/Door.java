// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoorConstants;

public class Door extends SubsystemBase {
  /** Creates a new Door. */

  private DigitalInput m_switchOpen; 
  private DigitalInput m_switchClose; 

  private WPI_TalonSRX m_motor;
  
  public Door() {
    m_switchOpen = new DigitalInput(DoorConstants.kOpenSwitch);
    m_switchClose = new DigitalInput(DoorConstants.kCloseSwitch);

    m_motor = new WPI_TalonSRX(DoorConstants.kDoorMotorID);

    this.setDefaultCommand(new RunCommand(() -> {moveDoor(0);}, this));
  }

  public boolean getOpenSwitch(){
    return m_switchOpen.get();
  }

  public boolean getCloseSwitch(){
    return m_switchClose.get();
  }

  public void moveDoor(double value){
    if ((m_switchOpen.get() && value > 0) || (m_switchClose.get() && value < 0)) {
      value = 0;
    }

    m_motor.set(value);
  }

  @Override
  public void periodic() {
    System.out.println("open: "+m_switchOpen.get()+"\nclose: "+m_switchClose.get());
  }
}
