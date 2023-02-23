// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.commands.TogglePipeline;
import frc.robot.subsystems.PhotonVision;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final Elevator m_elevatorSubsystem = new Elevator();

  private final Arm m_armSubsystem = new Arm();

  private final Intake m_intakeSubsystem = new Intake();

  private final PhotonVision m_photonVision = new PhotonVision();

  private final PhotonCamera m_camera = m_photonVision.getCamera();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand = new DriveArcade(m_driveTrain, m_driverController);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_chooser.setDefaultOption("first trajectory", null);
    m_chooser.addOption("second trajectory", m_driveArcadeCommand);

    SmartDashboard.putData("Routine selector", m_chooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // ELEVATOR:
    // up
    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(.1);
    }, m_elevatorSubsystem));

    // down
    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(-.1);
    }, m_elevatorSubsystem));

    // INTAKE:
    // get out
    m_driverController.y().whileTrue(new RunCommand(() -> {
      m_intakeSubsystem.lift(IntakeConstants.kLiftMotorSpeed);
    }, m_intakeSubsystem));

    // get in
    m_driverController.b().whileTrue(new RunCommand(() -> {
      m_intakeSubsystem.lift(-IntakeConstants.kLiftMotorSpeed);
    }, m_intakeSubsystem));

    // grab
    m_driverController.x().whileTrue(new RunCommand(() -> {
      m_intakeSubsystem.grab(0.7);
    }, m_intakeSubsystem));

    // ARM:
    // rotate up
    m_driverController.povUp().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(0.5);
    }, m_armSubsystem));

    // rotate down
    m_driverController.povDown().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(-0.5);
    }, m_armSubsystem));

    m_driverController.a().whileTrue(new RunCommand(() -> {
      m_armSubsystem.grabArm(0.1);
    }, m_armSubsystem));

    m_driverController.b().whileTrue(new RunCommand(() -> {
      m_armSubsystem.grabArm(-0.1);
    }, m_armSubsystem));

    m_driverController.a().onTrue(new TogglePipeline(m_photonVision, VisionConstants.kAprilPipline));
    m_driverController.back().onTrue(new TogglePipeline(m_photonVision, VisionConstants.kRetroPipline));
  }
}
