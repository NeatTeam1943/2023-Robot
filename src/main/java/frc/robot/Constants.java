// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Joystick
    public static final int kJoystickPort = 0; 
    public static final int kJoystickBButton = 2;
    // Wpi encoder 
    public static final int kResetEncoder = 0;
    // Autonomous command
    public static final double kAutoSpeed = .5; 
    // Simulataion values
    public static final double kUpdateTime = 0.02;
    public static final int kCountsPerRev = 4096;
    public static final double kWheelRadiusInches = 3;
    public static final double kSensorGearRatio = 1.0;
    public static final double kGearRatio = 10.71; 

    // Const methods
    public static int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotations = wheelRotations * kSensorGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }

    public static double nativeUnitsToDistanceMeters(double sensorCounts){
      double motorRotations = (double)sensorCounts / kCountsPerRev;
      double wheelRotations = motorRotations / kSensorGearRatio;
      double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
      return positionMeters;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final int kLeftFrontPort = 1;
    public static final int kLeftRearPort = 2;
    public static final int kRightFrontPort = 3;
    public static final int kRightRearPort = 4;
  }
}
