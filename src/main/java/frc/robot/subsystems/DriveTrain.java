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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private ADIS16448_IMUSim m_imuSim;

  private Field2d m_simField;

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

    m_imuSim = new ADIS16448_IMUSim(m_imu);

    m_driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(2), DriveTrainSimulation.kGearRatio, 2.1, 50, Units.inchesToMeters(DriveTrainSimulation.kWheelRadiusInches), 0.546, null);

    m_simField = new Field2d();
    SmartDashboard.putData("Field", m_simField);
  } 

  public void arcadeDrive(double move, double rot) {
    m_drive.arcadeDrive(move, rot);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Currently we don't have a configured gyro so the angle should always be 0, thats why we have a new object in the constructor
    double meterUnits = nativeUnitsToDistanceMeters(m_leftFront.getSelectedSensorPosition()); 
    double distance = nativeUnitsToDistanceMeters(m_rightFront.getSelectedSensorPosition());

    m_driveOdometry.update(doubleToRotation2d(m_imu.getAngle()), meterUnits, distance);
    
    m_simField.setRobotPose(m_driveOdometry.getPoseMeters());
  }

  public void simulationPeriodic(){
    m_leftFrontSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightFrontSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_leftRearSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightRearSim.setBusVoltage(RobotController.getBatteryVoltage());

    double leftMotorVolt = m_leftFrontSim.getMotorOutputLeadVoltage();
    double rightMotorVolt = m_rightFrontSim.getMotorOutputLeadVoltage();

    m_driveSim.setInputs(leftMotorVolt, -rightMotorVolt);
    
    m_driveSim.update(DriveTrainSimulation.kUpdateTime);

    m_leftFrontSim.setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters() * -1));
    m_leftFrontSim.setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));

    m_rightFrontSim.setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getRightPositionMeters() * -1));
    m_rightFrontSim.setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond()));
    
    m_leftRearSim.setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters() * -1));
    m_leftRearSim.setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));

    m_rightRearSim.setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getRightPositionMeters() * -1));
    m_rightRearSim.setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond() * -1));

    m_imuSim.setGyroAngleX(m_driveSim.getHeading().getDegrees());
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
  public static int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(DriveTrainSimulation.kWheelRadiusInches));
    double motorRotations = wheelRotations * DriveTrainSimulation.kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * DriveTrainSimulation.kCountsPerRev);
    return sensorCounts;
  }
  
  public static double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveTrainSimulation.kCountsPerRev;
    double wheelRotations = motorRotations / DriveTrainSimulation.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveTrainSimulation.kWheelRadiusInches));
    return positionMeters;
  }
  
  public static int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(DriveTrainSimulation.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * DriveTrainSimulation.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / DriveTrainSimulation.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveTrainSimulation.kCountsPerRev);
    return sensorCountsPer100ms;
  }  
}
