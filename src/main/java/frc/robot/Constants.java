// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final int kLeftFrontPort = 1;
    public static final int kLeftRearPort = 2;
    public static final int kRightFrontPort = 3;
    public static final int kRightRearPort = 4;
  }
  
  public static class DriveTrainSimulation{
    public static final double kUpdateTime = 0.02;
    public static final int kCountsPerRev = 4096;
    public static final double kWheelRadiusInches = 3;
    public static final double kSensorGearRatio = 1.0;
    public static final double kGearRatio = 10.71; 
    public static final int k100msPerSecond = 10;
  }
}
