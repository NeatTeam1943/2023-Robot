// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private WPI_TalonSRX m_rotateArmMotor;
  private WPI_TalonSRX m_grabArmMotor;

  private DigitalInput m_limitSwitchUp;
  private DigitalInput m_limitSwitchDown;

  private Color m_color;
  private Intake m_intake;

  public Arm() {
    m_rotateArmMotor = new WPI_TalonSRX(ArmConstants.kRotateArmMotorPort);
    m_grabArmMotor = new WPI_TalonSRX(ArmConstants.kGrabArmMotorPort);

    m_limitSwitchUp = new DigitalInput(ArmConstants.kLimitSwitchUpPort);
    m_limitSwitchDown = new DigitalInput(ArmConstants.kLimitSwitchDownPort);
  }

  public void rotateArm(double value){
    if (m_limitSwitchUp.get() && value > 0) {
      m_rotateArmMotor.set(0);
    } else if (m_limitSwitchDown.get() && value < 0) {
      m_rotateArmMotor.set(0);
    } else {
      m_rotateArmMotor.set(value);
    }
  }

  public void grabArm(double value) {
    m_color = m_intake.getColor();
    if (m_color == ArmConstants.kConeColor || m_color == ArmConstants.kCubeColor) {
      m_grabArmMotor.set(value);
    } else {
      m_grabArmMotor.set(-value);
    }
  }

  @Override
  public void periodic() {
  }
}