// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SensorConstants;

import com.revrobotics.ColorSensorV3;

public class Intake extends SubsystemBase {
  private Talon m_leftIntakeMotor;
  private Talon m_rightIntakeMotor;
  private Talon m_liftMotor;

  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;

  private ColorSensorV3 m_colorSensor;
  
  public Intake() {
    m_leftIntakeMotor = new Talon(IntakeConstants.kLeftIntakeMotorID);
    m_rightIntakeMotor = new Talon(IntakeConstants.kRightIntakeMotorID);
    m_liftMotor = new Talon(IntakeConstants.kLiftMotorID);

    m_topLimitSwitch = new DigitalInput(SensorConstants.kTopLimitSwitchPort);
    m_bottomLimitSwitch = new DigitalInput(SensorConstants.kBottomLimitSwitchPort);

    m_colorSensor = new ColorSensorV3(SensorConstants.kI2cPort);
  }

  public void grab(double speed) {
    m_leftIntakeMotor.set(speed);
    m_rightIntakeMotor.set(speed);
  }

  public void lift(double speed) {
    if (m_topLimitSwitch.get() && speed > 0)
      m_liftMotor.set(0);
    else if (m_bottomLimitSwitch.get() && speed < 0)
      m_liftMotor.set(0);
    else
      m_liftMotor.set(speed);
  }

  public Color getColor() {
    return m_colorSensor.getColor();
  }

  public int getProximity() {
    return m_colorSensor.getProximity();
  }

  public boolean isTopSwitchPressed(){
    return m_topLimitSwitch.get();
  }

  public boolean isBottomSwitchPressed(){
    return m_bottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
