// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX m_leftFront;
  private WPI_TalonFX m_leftRear;
  private WPI_TalonFX m_rightFront;
  private WPI_TalonFX m_rightRear;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  private DifferentialDrive m_drive;

  private ADIS16448_IMU m_imu;

  private Field2d m_field2d;

  private DifferentialDriveOdometry m_odometry;

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

    m_field2d = new Field2d();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_imu.getAngle()), 0, 0);
    
    m_driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(2), DriveTrainConstants.kGearRatio, 2.1, 25,
    Units.inchesToMeters(DriveTrainConstants.kWheelRadiusInches), 0.546, null);

    SmartDashboard.putData("field", m_field2d);
  }

  @Override
  public void periodic() {
    m_odometry.update(new Rotation2d(m_imu.getAngle()),
    nativeUnitsToDistanceMeters(m_leftFront.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(m_rightFront.getSelectedSensorPosition()));
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
  }
  
  public void arcadeDrive(double move, double rot, boolean squareInputs) {
    /*
     * move - robot movement speed across the x axis.
     * rot - robot rotation speed in the z axis.
     * squareInputs - If set to true, it decreases the input sensitivity at low speeds.
     */
    m_drive.arcadeDrive(move, rot, squareInputs);
  }

  public static double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / DriveTrainConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DriveTrainConstants.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveTrainConstants.kWheelRadiusInches));
    return positionMeters;
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

  public double getDistance() {
    double leftAvgPos = (m_leftFront.getSelectedSensorPosition() + m_leftRear.getSelectedSensorPosition()) / 2;
    double rightAvgPos = (m_rightFront.getSelectedSensorPosition() + m_rightRear.getSelectedSensorPosition()) / 2;
    double centralAvg = (leftAvgPos + rightAvgPos) / 2;
    double centralMotorAvg = centralAvg / DriveTrainConstants.kEncoderResolution;
    double centralWheelAvg = centralMotorAvg / DriveTrainConstants.kMotorToWheelRatio;

    return centralWheelAvg * DriveTrainConstants.kWheelCircumference;
  }

  public void resetEncoders() {
    m_leftFront.setSelectedSensorPosition(0);
    m_leftRear.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
    m_rightRear.setSelectedSensorPosition(0);
  }

  private double rawSpeedToRPM(double rawSpeed) {
    return rawSpeed * DriveTrainConstants.k100msTo60sRatio / DriveTrainConstants.kEncoderResolution
        / DriveTrainConstants.kMotorToWheelRatio;
  }

  public double getLeftWheelsRPM() {
    return rawSpeedToRPM((m_leftFront.getSelectedSensorVelocity() + m_leftRear.getSelectedSensorVelocity()) / 2);
  }

  public double getRightWheelsRPM() {
    return rawSpeedToRPM((m_rightFront.getSelectedSensorVelocity() + m_rightRear.getSelectedSensorVelocity()) / 2);
  }

  public double getAverageWheelsRPM() {
    return (getLeftWheelsRPM() + getRightWheelsRPM()) / 2;
  }
}
