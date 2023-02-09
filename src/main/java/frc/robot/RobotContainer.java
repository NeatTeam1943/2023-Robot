// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.commands.TogglePipeline;
import frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.Command;
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

  private final Elevator m_elevatorSubsystem = new Elevator();
  
  private final Arm m_armSubsystem = new Arm();

  private final Intake m_intakeSubsystem = new Intake();
  private final PhotonVision m_camera = new PhotonVision();

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand = new DriveArcade(m_driveTrain, m_driverController);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(0.5);
    }, m_elevatorSubsystem));

    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(-0.5);
    }, m_elevatorSubsystem));
    
    m_driverController.a().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(IntakeConstants.kLiftMotorSpeed)));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intakeSubsystem.lift(-IntakeConstants.kLiftMotorSpeed)));
    
    // rotate arm
    m_driverController.povUp().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(0.5);
    }, m_armSubsystem));
    m_driverController.povDown().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(-0.5);
    }, m_armSubsystem));

    // grab arm
    m_driverController.x().whileTrue(new RunCommand(() -> {
      m_armSubsystem.grabArm(0.1);
    }, m_armSubsystem));

    m_driverController.a().onTrue(new TogglePipeline(m_camera, VisionConstants.kAprilPipline));
    
    m_driverController.b().onTrue(new TogglePipeline(m_camera, VisionConstants.kRetroPipline));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
