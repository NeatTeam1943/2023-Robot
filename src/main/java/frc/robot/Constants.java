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
    // Speed controller ports
    public static final int kBackLeftMotor = 1;
    public static final int kBackFrontMotor = 2;
    public static final int kFrontLeftMotor = 3;
    public static final int kFrontRightMotor = 4;
    public static final int kElevatorMotorID = 5;
    // Joystick
    public static final int kJoystickPort = 0; 
    public static final int kJoystickBButton = 2;
    public static final int kJoystickAbutton = 3;
    public static final int kJoystickYbutton = 4;
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

    //PhotonVision constants
    public static final double k_CameraHeightMeters = Units.inchesToMeters(24);
    public static final double k_TargetHeightMeters = Units.feetToMeters(5);
    public static final double k_CameraPitchRadians = Units.degreesToRadians(0);
    public static final String k_CameraName = "photonvision";

    //PID constants
    public static final double k_LinearP = 0.1;
    public static final double k_LinearI = 0.1;
    public static final double k_LinearD = 0.1;
    public static final double k_AngularP= 0.1;
    public static final double k_AngularI = 0.1;
    public static final double k_AngularD = 0.1;

}
