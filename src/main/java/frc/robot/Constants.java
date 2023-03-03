// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final int kLeftFrontPort = 1;
    public static final int kLeftRearPort = 2;
    public static final int kRightFrontPort = 3;
    public static final int kRightRearPort = 4;

    // field2d sim
    public static final int kCountsPerRev = 4096;
    public static final double kWheelRadiusCm = 3 * 2.54;
    public static final double kSensorGearRatio = 1.0;
    public static final double kGearRatio = 1 / 24;
    public static final double kRobotMass = 45;

    // Encoder calculation for rate and distance
    public static final double k100msTo60sRatio = 600;
    public static final double kEncoderResolution = 2048;
    public static final double kMotorToWheelRatio = 4;
    public static final double kWheelCircumference = 47.879 / 100;
  }

  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = 89;
    public static final int kRightIntakeMotorID = 77;
    public static final int kLiftMotorID = 79;
    public static final double kLiftMotorSpeed = .5;
    public static final int kTopLimitSwitchPort = 8;
    public static final int kTopBottomLimitSwitchPort = 9;

  }

  public static class SensorConstants {
    public static final int kTopLimitSwitchPort = 25;
    public static final int kBottomLimitSwitchPort = 23;
    public static final I2C.Port kI2cPort = I2C.Port.kOnboard;
  }

  public static class ElevatorConstants {
    public static final int kElevatorFrontMotorID = 6;
    public static final int kElevatorRearMotorID = 5;
    public static final int kTopSwitchPort = 1;
    public static final int kBotomSwitchPort = 0;
  }

  public static class ArmConstants {
    public static final int kRotateArmTopID = 12;
    public static final int kRotateArmBottomID = 11;
    public static final int kGrabArmMotorID = 7;
    public static final int kLimitSwitchUpPort = 4;
    public static final int kLimitSwitchDownPort = 5;
    public static final Color kConeColor = new Color(0, 0, 0);
  }

  public static class VisionConstants {
    public static final String kCameraName = "Microsoft_LifeCam_HD-3000";
    public static final int kAprilPipline = 0;
    public static final int kRetroPipline = 1;
    public static final double kForwardP = 0;
    public static final double kForwardI = 0;
    public static final double kForwardD = 0;
    public static final double kCameraHeight = 1.15;
    public static final double kCameraPitchRadians = 0;
  }

  public static class TurnPIDConstants {
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kTurnToleranceDeg = 0;
    public static final double kTurnRateToleranceDegPerS = 0;
  }

  public static class AutoConstants {
    public static final double kSpeed = -0.4;

    // Long auto
    // public static final double kTime = 8;

    // Short auto
    // public static final double kTime = 3.5;

    // Charge
    public static final double kTime = 4;
    public static final double kChargeStationWidthMeters = 1.93;
    public static final double k_GridToChargeStationMeters = 1.44;
    public static final double k_DistanceToStable = 1;
  }
}
