// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonFX backLeftMotor; 
  private WPI_TalonFX backRightMotor;
  private WPI_TalonFX frontLeftMotor; 
  private WPI_TalonFX frontRightMotor;
  private DifferentialDrive differentialDrive;
  private edu.wpi.first.math.kinematics.DifferentialDriveOdometry DifferentialDriveOdometry;

  protected TalonFXSimCollection backLeftMotorSim;
  protected TalonFXSimCollection backRightMotorSim;
  protected TalonFXSimCollection frontLeftMotorSim;
  protected TalonFXSimCollection frontRightMotorSim;
  protected DifferentialDrivetrainSim differentialDriveSim;
  protected Field2d simField;

  
  public DriveTrain() {
    this.backLeftMotor = new WPI_TalonFX(Constants.kBackFrontMotor);
    this.backRightMotor = new WPI_TalonFX(Constants.kBackLeftMotor);
    this.frontLeftMotor = new WPI_TalonFX(Constants.kFrontLeftMotor);
    this.frontRightMotor = new WPI_TalonFX(Constants.kFrontRightMotor);

    this.frontLeftMotorSim = frontLeftMotor.getSimCollection();
    this.frontRightMotorSim = frontRightMotor.getSimCollection();

    this.backLeftMotor.follow(this.frontLeftMotor);
    this.backRightMotor.follow(this.frontRightMotor);

    this.differentialDrive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);
    this.DifferentialDriveOdometry = new edu.wpi.first.math.kinematics.DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    // I currently dont know to real parameters of our robot's driveTrain, these are temp variables 
    this.differentialDriveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(2), Constants.kGearRatio, 2.1, 26.5, Units.inchesToMeters(Constants.kWheelRadiusInches), 0.546, null);
    
    this.simField = new Field2d();
    
  }

  public WPI_TalonFX getBackLeftMotor(){return this.backLeftMotor;}

  public TalonFXSimCollection getbackLeftMotorSim(){return this.backLeftMotor.getSimCollection();}
  public TalonFXSimCollection getbackRightMotorSim(){return this.backRightMotor.getSimCollection();}
  public TalonFXSimCollection getfrontLeftMotorSim(){return this.frontLeftMotor.getSimCollection();}
  public TalonFXSimCollection getfrontRightMotorSim(){return this.frontRightMotor.getSimCollection();}
  public DifferentialDrivetrainSim getdifferentialDriveSim(){return this.differentialDriveSim;}
  
  public void robotArcadeDrive(double move, double rot){
    if (move < 0.05) {move = 0;}
    if (rot < 0.05) {rot = 0;}
    differentialDrive.arcadeDrive(move, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Currently we don't have a configured gyro so the angle should always be 0, thats why we have a new object in the constructor
    DifferentialDriveOdometry.update(new Rotation2d(), Constants.nativeUnitsToDistanceMeters(this.frontLeftMotor.getSelectedSensorPosition()),
    Constants.nativeUnitsToDistanceMeters(this.frontLeftMotor.getSelectedSensorPosition()));
    simField.setRobotPose(DifferentialDriveOdometry.getPoseMeters());
  }

  public void simulationInit(){}

  public void simulationPeriodic(){
    RobotContainer.driveArcade.getfrontLeftMotorSim().setBusVoltage(RobotController.getBatteryVoltage());
    RobotContainer.driveArcade.getfrontRightMotorSim().setBusVoltage(RobotController.getBatteryVoltage());

    RobotContainer.driveArcade.getdifferentialDriveSim().setInputs(RobotContainer.driveArcade.getfrontLeftMotorSim().getMotorOutputLeadVoltage(), RobotContainer.driveArcade.getfrontRightMotorSim().getMotorOutputLeadVoltage() * -1);

    RobotContainer.driveArcade.getdifferentialDriveSim().update(Constants.kUpdateTime);
  }
}
