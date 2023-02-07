// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainSimulation;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX m_leftFront;
  private WPI_TalonFX m_leftRear;
  private WPI_TalonFX m_rightFront;
  private WPI_TalonFX m_rightRear;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  private DifferentialDrive m_drive;

  private ADIS16448_IMU m_imu;

  /*simulation */
  private TalonFXSimCollection m_leftFrontSim; 
  private TalonFXSimCollection m_leftRearSim;
  private TalonFXSimCollection m_rightFrontSim;
  private TalonFXSimCollection m_rightRearSim;;

  private edu.wpi.first.math.kinematics.DifferentialDriveOdometry m_driveOdometry;

  private DifferentialDrivetrainSim m_driveSim;

  public DriveTrain() {

    m_leftFront = new WPI_TalonFX(DriveTrainConstants.kLeftFrontPort);
    m_leftRear = new WPI_TalonFX(DriveTrainConstants.kLeftRearPort);
    m_rightFront = new WPI_TalonFX(DriveTrainConstants.kRightFrontPort);
    m_rightRear = new WPI_TalonFX(DriveTrainConstants.kRightRearPort);

    m_leftMotors = new MotorControllerGroup(m_leftRear, m_leftFront);
    m_rightMotors = new MotorControllerGroup(m_rightRear, m_rightFront);

    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_imu = new ADIS16448_IMU();

    
    /*simulation */
    m_leftFrontSim = m_leftFront.getSimCollection();
    m_leftFrontSim = m_leftRear.getSimCollection();
    m_leftFrontSim = m_rightFront.getSimCollection();
    m_leftFrontSim = m_rightRear.getSimCollection();

    m_driveOdometry = new DifferentialDriveOdometry(doubleToRotation2d(m_imu.getAngle()), 0, 0);

    m_driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(2), DriveTrainSimulation.kGearRatio, 2.1, 50, Units.inchesToMeters(DriveTrainSimulation.kWheelRadiusInches), 0.546, null);
  }

  public void arcadeDrive(double move, double rot) {
    m_drive.arcadeDrive(move, rot);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public double getHeading() {
    return m_imu.getAngle();
  }

  public double getRotationRate() {
    return m_imu.getRate();
  }

  public ADIS16448_IMU getIMU() {
    return m_imu;
  }

  public void calibrateIMU() {
    m_imu.calibrate();
  }
  // temp function. it should function like the getRotation2d function from ctre
  public Rotation2d doubleToRotation2d(double deg){
    return Rotation2d.fromDegrees(deg);
  }
}
