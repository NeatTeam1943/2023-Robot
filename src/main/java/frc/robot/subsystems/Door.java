// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoorConstants;

public class Door extends SubsystemBase {
  private WPI_TalonSRX m_door;

  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;


  /** Creates a new Door. */
  public Door() {
    m_door = new WPI_TalonSRX(DoorConstants.kDoorMotorID);
    m_topLimitSwitch = new DigitalInput(DoorConstants.kTopSwitchPort);
    m_bottomLimitSwitch = new DigitalInput(DoorConstants.kBottomSwitchPort);
    this.setDefaultCommand(new RunCommand(() -> {
      MoveDoor(DoorConstants.m_doorSpeed);
    },this));
  }

  public boolean isTopSwitchPressed(){
    return m_topLimitSwitch.get();
  }

  public boolean isBottomSwitchPressed(){
    return m_bottomLimitSwitch.get();
  }

  public void MoveDoor(double speed){
    if((isTopSwitchPressed() && speed > 0) || (isBottomSwitchPressed() && speed < 0)){
      m_door.set(0);
    }
    else if(speed > 0){
      m_door.set(speed);
    }
    else if(speed < 0){
      m_door.set(-speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
