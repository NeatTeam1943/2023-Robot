// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private WPI_TalonSRX m_rotateArmMotor;
  private WPI_TalonSRX m_grabArmMotor;

  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;

  private Color m_color;
  private Intake m_intake;

  public Arm() {
    m_rotateArmMotor = new WPI_TalonSRX(ArmConstants.kRotateArmMotorID);
    m_grabArmMotor = new WPI_TalonSRX(ArmConstants.kGrabArmMotorID);

    m_rotateArmMotor.setNeutralMode(NeutralMode.Brake);
    
    m_topLimitSwitch = new DigitalInput(ArmConstants.kLimitSwitchUpPort);
    m_bottomLimitSwitch = new DigitalInput(ArmConstants.kLimitSwitchDownPort);

    this.setDefaultCommand(new RunCommand(() -> {
      rotateArm(0);
      grabArm(0);
    }, this));
  }

  public void rotateArm(double value) {
    if ((m_topLimitSwitch.get() && value > 0) || (m_bottomLimitSwitch.get() && value < 0)) {
      value = 0;
    }

    m_rotateArmMotor.set(value);
  }

  public void grabArm(double value) {
    m_grabArmMotor.set(value);
  }

  @Override
  public void periodic() {
  }
}
