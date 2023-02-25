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

    // Encoder calculation for rate and distance
    public static final double k100msTo60sRatio = 600;
    public static final double kEncoderResolution = 2048;
    public static final double kMotorToWheelRatio = 4;
    public static final double kWheelCircumference = 47.879;
  }

  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = -1;
    public static final int kRightIntakeMotorID = -1;
    public static final int kLiftMotorID = -1;
    public static final double kLiftMotorSpeed = .5;
  }

  public static class SensorConstants {
    public static final int kTopLimitSwitchPort = 0;
    public static final int kBottomLimitSwitchPort = 0;
    public static final I2C.Port kI2cPort = I2C.Port.kOnboard;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotor = 5;
    public static final int kTopSwitchPort = 0;
    public static final int kBotomSwitchPort = 1;
  }

  public static class ArmConstants {
    public static final int kRotateArmMotorPort = 5;
    public static final int kGrabArmMotorPort = 6;
    public static final int kLimitSwitchUpPort = 2;
    public static final int kLimitSwitchDownPort = 3;
    public static final Color kCubeColor = new Color(0, 0, 0);
    public static final Color kConeColor = new Color(0, 0, 0);
  }

  public static class VisionConstants {
    public static final String kCameraName = "photonvision";
    public static final int kAprilPipline = 0;
    public static final int kRetroPipline = 1;
    public static final double kCameraHeight = 69;
    public static final double kCameraPitchRadians = 69;
    public static final double kRetroReflectiveHeight = 69;
    public static final double kAprilTagHeight = 69;
  }
  
  public static class PIDConstants {
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kForwardP = 0;
    public static final double kForwardI = 0;
    public static final double kForwardD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kTurnToleranceDeg =0;
    public static final double kTurnRateToleranceDegPerS = 0;
  }

  public static class ColorSensorConstants {
    public static Color kCone = new Color(69, 69, 69);
  }
}

