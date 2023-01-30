// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  static public class JoystickConstants {
    public static final int kJoystickPort = 0;
    public static final int kJoystickBButton = 2;
    public static final int kJoystickAbutton = 3;
    public static final int kJoystickYbutton = 4;
  }

  static public class ElevatorConstants {
    public static final int kElevatorMotorID = 5;
  }
}
