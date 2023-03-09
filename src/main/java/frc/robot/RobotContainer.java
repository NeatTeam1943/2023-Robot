// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutonomousNames;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.auto.Climb;
import frc.robot.commands.auto.DriveToChargeStaion;
import frc.robot.commands.auto.DriveToCommunity;
import frc.robot.commands.auto.Stabilize;
import frc.robot.commands.auto.DriveMeters;
import frc.robot.commands.door.CloseDoor;
import frc.robot.commands.door.OpenDoor;
import frc.robot.subsystems.Door;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  private final DriveTrain m_driveTrain = new DriveTrain();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand = new DriveArcade(m_driveTrain, m_driverController);

  private final Door m_door = new Door();

  public RobotContainer() {
    SmartDashboard.putStringArray("Auto List", new String[] { AutonomousNames.kdriveMeters,
        AutonomousNames.kdriveToComunityAndStabilize, AutonomousNames.kjustStabilize });

    m_driveTrain.setDefaultCommand(m_driveArcadeCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  private void configureBindings() {
    // Open door
    m_driverController.a().onTrue(new OpenDoor(m_door));

    // Close door
    m_driverController.a().onFalse(new CloseDoor(m_door));
  }

  public Command leaveCommunityAndStabilze() {
    OpenDoor open = new OpenDoor(m_door);
    CloseDoor close = new CloseDoor(m_door);

    DriveToCommunity driveToCommunity = new DriveToCommunity(m_driveTrain, true);
    Command driveToCommunityAndCloseDoor = Commands.parallel(driveToCommunity, close);

    DriveToChargeStaion driveToChargeStaion = new DriveToChargeStaion(m_driveTrain, false);
    Climb climb = new Climb(m_driveTrain, false);
    Stabilize stabilize = new Stabilize(m_driveTrain);

    return Commands.sequence(open, driveToCommunityAndCloseDoor, driveToChargeStaion, climb, stabilize);
  }

  public Command driveMeters() {
    OpenDoor open = new OpenDoor(m_door);

    CloseDoor close = new CloseDoor(m_door);
    DriveMeters driveOut = new DriveMeters(m_driveTrain, 3, true);

    return Commands.sequence(open, driveOut, close);
  }

  public Command justStabilize() {
    OpenDoor open = new OpenDoor(m_door);
    CloseDoor close = new CloseDoor(m_door);

    DriveToChargeStaion driveToChargeStaion = new DriveToChargeStaion(m_driveTrain, true);
    Climb climb = new Climb(m_driveTrain, true);
    Stabilize stabilize = new Stabilize(m_driveTrain);

    return Commands.sequence(open, driveToChargeStaion, close, climb, stabilize);
  }

  public Command getAuto() {
    String autoName = SmartDashboard.getString("Auto Selector", AutonomousNames.kdriveMeters);
    switch (autoName) {
      case AutonomousNames.kdriveToComunityAndStabilize:
        System.out.println("Driving to community and stabilizing...");
        return leaveCommunityAndStabilze();

      case AutonomousNames.kjustStabilize:
        System.out.println("Stabilizing...");
        return justStabilize();

      case AutonomousNames.kdriveMeters:
        System.out.println("Exiting community...");
        return new DriveToCommunity(m_driveTrain, false);
    }

    return null; 
  }
}