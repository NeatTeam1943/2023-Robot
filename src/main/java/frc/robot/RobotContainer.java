// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveArcade;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SimDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final SimDrive m_simDrive = new SimDrive();
  private final Elevator m_elevatorSubsystem = new Elevator();

  // The robot's subsystems and commands are defined here...
  private final Intake m_intakeSubsystem = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand = new DriveArcade(m_driveTrain, m_driverController);
  private final DriveArcade m_driveArcadeCommandSim = new DriveArcade(m_simDrive, m_driverController);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_simDrive.setDefaultCommand(m_driveArcadeCommandSim);
    m_driveTrain.setDefaultCommand(m_driveArcadeCommand);
  }

  private void configureBindings() {
    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(0.5);
    }, m_elevatorSubsystem));

    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(-0.5);
    }, m_elevatorSubsystem));

    m_driverController.a().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(IntakeConstants.kLiftMotorSpeed)));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(-IntakeConstants.kLiftMotorSpeed)));
    m_driverController.b().whileTrue(new RunCommand(() -> m_simDrive.resetRobotPosition()));
  }

  public Command getAutonomousCommand() {
    return new AutoDrive(m_simDrive);
  }
}
