// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerGripper extends SubsystemBase {
  private WPI_TalonFX m_rollerGripper;

  /** Creates a new IntakeArm. */
  public RollerGripper() {
    m_rollerGripper = new WPI_TalonFX(IntakeConstants.kRollerGripperMotorID);
  }

  public void grip(double power){
    m_rollerGripper.set(power);
  }
  
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
