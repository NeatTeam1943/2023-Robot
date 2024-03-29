// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  public DriveTrain() {
    m_leftFront = new WPI_TalonFX(DriveTrainConstants.kLeftFrontPort);
    m_leftRear = new WPI_TalonFX(DriveTrainConstants.kLeftRearPort);
    m_rightFront = new WPI_TalonFX(DriveTrainConstants.kRightFrontPort);
    m_rightRear = new WPI_TalonFX(DriveTrainConstants.kRightRearPort);

    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftRear.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightRear.setNeutralMode(NeutralMode.Brake);

    resetEncoders();

    m_rightFront.setInverted(true);
    m_rightRear.setInverted(true);

    m_leftMotors = new MotorControllerGroup(m_leftRear, m_leftFront);
    m_rightMotors = new MotorControllerGroup(m_rightRear, m_rightFront);

    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_imu = new ADIS16448_IMU();

    m_field2d = new Field2d();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_imu.getAngle()), 0, 0);

    SmartDashboard.putData("field", m_field2d);
  }

  @Override
  public void periodic() {
    m_odometry.update(
        new Rotation2d(m_imu.getAngle()),
        nativeUnitsToDistanceMeters(
            m_leftFront.getSelectedSensorPosition()), // Should fix and add acutal angle value
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
    double positionMeters =
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveTrainConstants.kWheelRadiusCm));
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

  public double getGyroAngleY() {
    return m_imu.getGyroAngleY();
  }

  public boolean isStable() {
    final double angleY = getGyroAngleY();
    return -1 < angleY && angleY < 1;
  }

  public ADIS16448_IMU getIMU() {
    return m_imu;
  }

  public void calibrateIMU() {
    m_imu.reset();
  }

  public double getDistance() {
    double leftAvgPos =
        (m_leftFront.getSelectedSensorPosition() + m_leftRear.getSelectedSensorPosition()) / 2;
    double rightAvgPos =
        (m_rightFront.getSelectedSensorPosition() + m_rightRear.getSelectedSensorPosition()) / 2;
    double centralAvg = (leftAvgPos + rightAvgPos) / 4;
    double centralMotorAvg = centralAvg / DriveTrainConstants.kEncoderResolution;
    double centralWheelAvg = centralMotorAvg / DriveTrainConstants.kMotorToWheelRatio;

    return (-centralWheelAvg * DriveTrainConstants.kWheelCircumference) / 100;
  }

  public void resetEncoders() {
    m_leftFront.setSelectedSensorPosition(0);
    m_leftRear.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
    m_rightRear.setSelectedSensorPosition(0);
  }

  private double rawSpeedToRPM(double rawSpeed) {
    return rawSpeed
        * DriveTrainConstants.k100msTo60sRatio
        / DriveTrainConstants.kEncoderResolution
        / DriveTrainConstants.kMotorToWheelRatio;
  }

  public double getLeftWheelsRPM() {
    return rawSpeedToRPM(
        (m_leftFront.getSelectedSensorVelocity() + m_leftRear.getSelectedSensorVelocity()) / 2);
  }

  public double getRightWheelsRPM() {
    return rawSpeedToRPM(
        (m_rightFront.getSelectedSensorVelocity() + m_rightRear.getSelectedSensorVelocity()) / 2);
  }

  public double getAverageWheelsRPM() {
    return (getLeftWheelsRPM() + getRightWheelsRPM()) / 2;
  }
}
