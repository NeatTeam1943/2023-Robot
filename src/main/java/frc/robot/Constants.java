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
    public static final double kWheelCircumference = 47.879;
  }

  public static class DoorConstants {
    public static final int kDoorMotorID = 5;

    public static final int kCloseSwitch = 0;
    public static final int kOpenSwitch = 1;

    public static final double kDoorSpeed = 0.15;

    public static final double kCloseDoorDelay = 0.2;
  }

  public static final class CommunityConstants {
    public static final double kShortDistance = 2.2;
    public static final double kLongDistance = 3.8;
    public static final double kLineDistance = 1;
  }
  }
}
