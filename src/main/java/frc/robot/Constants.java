// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final int kLeftFrontPort = 1;
    public static final int kLeftRearPort = 2;
    public static final int kRightFrontPort = 3;
    public static final int kRightRearPort = 4;

    // Encoder calculation for rate and distance
    public static final double k100msTo60sRatio = 600;
    public static final double kEncoderResolution = 2048;
    public static final double kMotorToWheelRatio = 4;
    public static final double kWheelCircumference = 47.879;
  }

  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = 1;
    public static final int kRightIntakeMotorID = 2;
    public static final int kLiftMotorID = 3;
    public static final double kLiftMotorSpeed = .5;
  }

  public static class SensorConstants {
    public static final int kTopLimitSwitchPort = 1;
    public static final int kBottomLimitSwitchPort = 2;
    public static final I2C.Port kI2cPort = I2C.Port.kOnboard;

    public static class DriveSimulation {
      public static final double kUpdateTime = 0.02;
      public static final int kCountsPerRev = 4096;
      public static final double kWheelRadiusInches = 3;
      public static final double kSensorGearRatio = 1.0;
      public static final double kGearRatio = 10.71;
      public static final int k100msPerSecond = 10;

      public static final int kFrontRightMotor = 1;
      public static final int kBackRightMotor = 3;
      public static final int kFrontLeftMotor = 2;
      public static final int kBackLeftMotor = 4;
    }
  }
  public static class ElevatorConstants{
    public static final int kElevatorMotor = 5;
    public static final int kTopSwitchPort = 0;
    public static final int kBotomSwitchPort = 1;
  }
}
