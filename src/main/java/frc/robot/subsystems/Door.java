// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoorConstants;

public class Door extends SubsystemBase {
  private Servo m_door;


  /** Creates a new Door. */
  public Door() {
    m_door = new Servo(DoorConstants.kDoor1Channel);
    m_door = new Servo(DoorConstants.kDoor2Channel);
    this.setDefaultCommand(new RunCommand(() -> {
      MoveDoor(0, 0);
    },this));
  }

  public void MoveDoor(double value, double angle){
    m_door.set(value);
    m_door.setAngle(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
