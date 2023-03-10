// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutonomousNames;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.door.CloseDoor;
import frc.robot.commands.door.OpenDoor;
import frc.robot.subsystems.Door;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    SmartDashboard.putStringArray("Auto List", new String[] {
        AutonomousNames.kPassLine,
        AutonomousNames.kPassShort,
        AutonomousNames.kPassLong,
        AutonomousNames.kPassThroughCharge,
        AutonomousNames.kStabilize,
        AutonomousNames.kPassNStable,
        AutonomousNames.kGamePieceOnly,
        AutonomousNames.kGamePieceNStable,
        AutonomousNames.kGamePieceNShort,
        AutonomousNames.kGamePieceNLong,
        AutonomousNames.kFullRoute,
        AutonomousNames.kDoNothing,
    });

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

  public Command getAuto() {
    AutoFunctions autos = new AutoFunctions(m_driveTrain, m_door);
    String autoName = SmartDashboard.getString("Auto Selector", AutonomousNames.kDoNothing);

    switch (autoName) {
      // Pass
      case AutonomousNames.kPassLine:
        return autos.passLine();

      case AutonomousNames.kPassShort:
        return autos.passShort();

      case AutonomousNames.kPassLong:
        return autos.passLong();

      case AutonomousNames.kPassThroughCharge:
        return autos.passChargeStation();

      // Stabilize
      case AutonomousNames.kStabilize:
        return autos.stabilize(true);

      // Game piece
      case AutonomousNames.kGamePieceOnly:
        return autos.gamePiece();

      case AutonomousNames.kGamePieceNStable:
        return autos.gamePieceAndStabilize();

      case AutonomousNames.kGamePieceNShort:
        return autos.gamePieceAndPassShort();

      case AutonomousNames.kGamePieceNLong:
        return autos.gamePieceAndPassLong();

      // Perform full routine
      case AutonomousNames.kFullRoute:
        return autos.fullRoute();
    }

    return null;
  }
}