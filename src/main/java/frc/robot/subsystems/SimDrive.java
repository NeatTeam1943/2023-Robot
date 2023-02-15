// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants.DriveSimulation;

public class SimDrive extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonFX m_rearLeft;
  private WPI_TalonFX m_rearRight;
  private WPI_TalonFX m_frontLeft;
  private WPI_TalonFX m_frontRight;
  private MotorControllerGroup m_leftMotorGroup;
  private MotorControllerGroup m_rightGroup;
  private WPI_PigeonIMU m_pigeon;
  private DifferentialDrive m_drive;
  private edu.wpi.first.math.kinematics.DifferentialDriveOdometry m_odometry;
  private TalonFXSimCollection m_frontLeftSim;
  private TalonFXSimCollection m_frontRightSim;
  private TalonFXSimCollection m_rearLeftSim;
  private TalonFXSimCollection m_rearRightSim;
  private BasePigeonSimCollection m_pigeonSim;
  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_simField;

  private Pose2d m_resetLocation; 
  

  private double m_gyroOffset;

  public SimDrive() {
    m_rearLeft = new WPI_TalonFX(DriveSimulation.kBackLeftMotor);
    m_rearRight = new WPI_TalonFX(DriveSimulation.kBackRightMotor);
    m_frontLeft = new WPI_TalonFX(DriveSimulation.kFrontLeftMotor);
    m_frontRight = new WPI_TalonFX(DriveSimulation.kFrontRightMotor);

    m_rearLeftSim = m_frontLeft.getSimCollection();
    m_rearRightSim = m_frontRight.getSimCollection();
    m_frontLeftSim = m_rearLeft.getSimCollection();
    m_frontRightSim = m_rearRight.getSimCollection();

    m_pigeon = new WPI_PigeonIMU(0);
    m_pigeonSim = m_pigeon.getSimCollection();

    m_leftMotorGroup = new MotorControllerGroup(m_rearLeft, m_frontLeft);
    m_rightGroup = new MotorControllerGroup(m_rearRight, m_frontRight);

    m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);
    m_odometry = new edu.wpi.first.math.kinematics.DifferentialDriveOdometry(
        m_pigeon.getRotation2d(), 0, 0);

    m_driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(2), DriveSimulation.kGearRatio, 2.1, 25,
        Units.inchesToMeters(DriveSimulation.kWheelRadiusInches), 0.546, null);

    m_simField = new Field2d();
    m_resetLocation = m_simField.getRobotPose();
    SmartDashboard.putData("Field", m_simField);
  }

  public WPI_TalonFX getM_rearLeft() {
    return m_rearLeft;
  }

  public TalonFXSimCollection getbackLeftMotorSim() {
    return m_rearLeft.getSimCollection();
  }

  public TalonFXSimCollection getbackRightMotorSim() {
    return m_rearRight.getSimCollection();
  }

  public TalonFXSimCollection getfrontLeftMotorSim() {
    return m_frontLeft.getSimCollection();
  }

  public TalonFXSimCollection getfrontRightMotorSim() {
    return m_frontRight.getSimCollection();
  }

  public DifferentialDrivetrainSim getdifferentialDriveSim() {
    return m_driveSim;
  }

  public WPI_PigeonIMU getM_pigeon() {
    return m_pigeon;
  }

  public void robotArcadeDrive(double move, double rot) {
    m_drive.arcadeDrive(-move, rot);
  }

  public void robotTankDrive(double leftMove, double rightMoveStick) {
    m_drive.tankDrive(leftMove, rightMoveStick);
  }

  public static int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(DriveSimulation.kWheelRadiusInches));
    double motorRotations = wheelRotations * DriveSimulation.kSensorGearRatio;
    int sensorCounts = (int) (motorRotations * DriveSimulation.kCountsPerRev);
    return sensorCounts;
  }

  public static double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / DriveSimulation.kCountsPerRev;
    double wheelRotations = motorRotations / DriveSimulation.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveSimulation.kWheelRadiusInches));
    return positionMeters;
  }

  public static int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond
        / (2 * Math.PI * Units.inchesToMeters(DriveSimulation.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * DriveSimulation.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / DriveSimulation.k100msPerSecond;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * DriveSimulation.kCountsPerRev);
    return sensorCountsPer100ms;
  }

  public void setPigeon(double position) {
    m_gyroOffset = (position - m_pigeon.getFusedHeading());
  }

  public double getRotation() {
    return m_pigeon.getFusedHeading() + m_gyroOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Currently we don't have a configured gyro so the angle should always be 0,
    // thats why we have a new object in the constructor
    m_odometry.update(m_pigeon.getRotation2d(),
        nativeUnitsToDistanceMeters(m_frontLeft.getSelectedSensorPosition()),
        nativeUnitsToDistanceMeters(m_frontRight.getSelectedSensorPosition()));
    m_simField.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetRobotPosition(){
    m_simField.setRobotPose(0, 0, m_pigeon.getRotation2d());
    m_frontLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
    m_pigeon.reset();
    m_odometry.update(m_pigeon.getRotation2d(),
    0, 0);
    m_odometry.resetPosition(m_pigeon.getRotation2d(), 0, 0, m_resetLocation);
  }

  public void simulationInit() {
  }
  
  public void simulationPeriodic() {
    m_rearLeftSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rearRightSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_frontLeftSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_frontRightSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_driveSim.setInputs(
    m_rearLeftSim.getMotorOutputLeadVoltage(),
    m_rearRightSim.getMotorOutputLeadVoltage() * -1); 

    m_driveSim.update(DriveSimulation.kUpdateTime);

    m_rearLeftSim
        .setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters() * -1));
    m_rearLeftSim
        .setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));

    m_rearRightSim
        .setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getRightPositionMeters() * -1));
    m_rearRightSim.setIntegratedSensorVelocity(
        velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond()));

    m_frontLeftSim
        .setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters() * -1));
    m_frontLeftSim
        .setIntegratedSensorVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));

    m_frontRightSim
        .setIntegratedSensorRawPosition(distanceToNativeUnits(m_driveSim.getRightPositionMeters() * -1));
    m_frontRightSim.setIntegratedSensorVelocity(
        velocityToNativeUnits(m_driveSim.getRightVelocityMetersPerSecond() * -1));

    m_pigeonSim.setRawHeading(m_driveSim.getHeading().getDegrees());
  }
}