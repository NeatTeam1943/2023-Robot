// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.crypto.KeySelector.Purpose;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonFX backLeftMotor; 
  private WPI_TalonFX backRightMotor;
  private WPI_TalonFX frontLeftMotor; 
  private WPI_TalonFX frontRightMotor;
  
  private DifferentialDrive differentialDrive;


  public DriveTrain() {
    this.backLeftMotor = new WPI_TalonFX(Constants.kBackFrontMotor);
    this.backRightMotor = new WPI_TalonFX(Constants.kBackLeftMotor);
    this.frontLeftMotor = new WPI_TalonFX(Constants.kFrontLeftMotor);
    this.frontRightMotor = new WPI_TalonFX(Constants.kFrontRightMotor);

    this.backLeftMotor.follow(this.frontLeftMotor);
    this.backRightMotor.follow(this.frontRightMotor);

    this.differentialDrive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);
  }

  public void robotArcadeDrive(double move, double rot){
    if (move < 0.05) {move = 0;}
    if (rot < 0.05) {rot = 0;}
    differentialDrive.arcadeDrive(move, rot);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
