// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousNames;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.door.CloseDoor;
import frc.robot.commands.door.OpenDoor;
import frc.robot.subsystems.Door;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain m_driveTrain = new DriveTrain();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand =
      new DriveArcade(m_driveTrain, m_driverController);

  private final Door m_door = new Door();

  public RobotContainer() {
    SmartDashboard.putStringArray(
        "Auto List",
        new String[] {
          AutonomousNames.kPassLine,
          AutonomousNames.kPassShort,
          AutonomousNames.kPassLong,
          AutonomousNames.kPassThroughCharge,
          AutonomousNames.kStabilize,
          AutonomousNames.kPassNStable,
          AutonomousNames.kGamePieceOnly,
          AutonomousNames.kGamePieceNStable,
          AutonomousNames.kGamePieceNShort,
          AutonomousNames.kGamePieceNPassChargeStation,
          AutonomousNames.kGamePieceNLong,
          AutonomousNames.kFullRoute,
          AutonomousNames.kDoNothing,
        });

    m_driveTrain.setDefaultCommand(m_driveArcadeCommand);
    m_door.setDefaultCommand(new CloseDoor(m_door)); 

    // Configure the trigger bindingsp
    configureBindings();

  }

  public void resetDriveTrain() {
    m_driveTrain.resetEncoders();
    m_driveTrain.calibrateIMU();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Open door
    m_driverController.a().onTrue(new OpenDoor(m_door));

    // Close door
    m_driverController.a().onFalse(new CloseDoor(m_door));
  }

  public double getGyroAngleY() {
    return m_driveTrain.getGyroAngleY();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoFunctions autos = new AutoFunctions(m_driveTrain, m_door);
    String autoName = SmartDashboard.getString("Auto Selector", AutonomousNames.kDoNothing);

    System.out.print("Choosen autonomous: ");
    switch (autoName) {
        // Pass
      case AutonomousNames.kPassLine:
        System.out.println("Pass line");
        return autos.passLine();

      case AutonomousNames.kPassShort:
        System.out.println("Pass line on short distance");
        return autos.passShort();

      case AutonomousNames.kPassLong:
        System.out.println("Pass line on long distance");
        return autos.passLong();

      case AutonomousNames.kPassThroughCharge:
        System.out.println("Pass charge station");
        return autos.passChargeStation();

        // Stabilize
      case AutonomousNames.kStabilize:
        System.out.println("Stabilize");
        return autos.stabilize(true);

      case AutonomousNames.kPassNStable:
        System.out.println("Pass charge station and stabilize");
        return autos.passChargeStationAndStabilize();

        // Game piece
      case AutonomousNames.kGamePieceOnly:
        System.out.println("Score");
        return autos.gamePiece();

      case AutonomousNames.kGamePieceNStable:
        System.out.println("Score and stabilize");
        return autos.gamePieceAndStabilize();

      case AutonomousNames.kGamePieceNShort:
        System.out.println("Score and pass line on short distance");
        return autos.gamePieceAndPassShort();

      case AutonomousNames.kGamePieceNLong:
        System.out.println("Score and pass line on long distance");
        return autos.gamePieceAndPassLong();

      case AutonomousNames.kGamePieceNPassChargeStation:
        System.out.println("Score and pass charge");
        return autos.gamePieceAndPassChargeStation();

        // Perform full routine
      case AutonomousNames.kFullRoute:
        System.out.println("Score, pass line trough charge station and stabilize");
        return autos.fullRoute();
    }

    System.out.println("Nothing");
    return null;
  }
}
