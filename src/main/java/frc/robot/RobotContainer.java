// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SimDrive;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // configureBindings();
    m_simDrive.setDefaultCommand(m_driveArcadeCommandSim);
    m_driveTrain.setDefaultCommand(m_driveArcadeCommand);
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
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(0.5);
    }, m_elevatorSubsystem));

    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(-0.5);
    }, m_elevatorSubsystem));
    
    m_driverController.a().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(IntakeConstants.kLiftMotorSpeed)));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(-IntakeConstants.kLiftMotorSpeed)));
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

