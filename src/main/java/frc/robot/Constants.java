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

  public static final class AutoDriveConstans {
    public static final double kDefaultSpeed = 0.25;
  }

  public static final class StabilizeConstants {
    public static final double kSlowSpeed = 0.097;
    public static final double kAngleThreshold = 3.5;
  }

  public static final class ChargeStationConstans {
    public static final double kApproachSpeed = 0.15;
    public static final double kClimbAngleThreshold = 12;
    public static final double kClimbDistance = 0.93;
  }

  public static final class PassChargeStationConstants {
    public static final double kUpSpeed = 0.2;
    public static final double kDownSpeed = 0.15;
    public static final double kUnsafeDistance = 4.25;
    public static final double kSafeDistance = 0.25;
  }

  public static final class AutonomousNames {
    public static final String kPassLine = "pass line";
    public static final String kPassShort = "pass short";
    public static final String kPassLong = "pass long";
    public static final String kPassThroughCharge = "pass through charge";

    public static final String kStabilize = "stabilize only";
    public static final String kPassNStable = "pass & stabilize";

    public static final String kGamePieceOnly = "game piece only";

    public static final String kGamePieceNStable = "game piece & stable";
    public static final String kGamePieceNShort = "game piece & pass short";
    public static final String kGamePieceNLong = "game piece & pass long";
    public static final String kGamePieceNPassChargeStation = "game piece & pass charge station";

    public static final String kFullRoute = "full route";

    public static final String kDoNothing = "nothing";
  }
}
