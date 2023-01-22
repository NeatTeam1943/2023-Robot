// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private CANSparkMax elevatorMotor;

  public Elevator() {
    this.elevatorMotor = new CANSparkMax(Constants.kElevatorMotorID, MotorType.kBrushed);
  }

  public void elevate(double speed){
    this.elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
