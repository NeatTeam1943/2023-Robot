// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DoorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveToChargeStaion;
import frc.robot.commands.DriveToCommunity;
import frc.robot.commands.Stabilize;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Door;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final Elevator m_elevatorSubsystem = new Elevator();

  private final Arm m_armSubsystem = new Arm();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveArcade m_driveArcadeCommand = new DriveArcade(m_driveTrain, m_driverController);

  public RobotContainer() {
    m_driveTrain.setDefaultCommand(m_driveArcadeCommand);
    //m_driveTrain.calibrateIMU();

    // Configure the trigger bindings
    configureBindings();
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  private void configureBindings() {
    // ELEVATOR:
    // up
    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(.3);
    }, m_elevatorSubsystem));

    // down
    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_elevatorSubsystem.moveElevator(-.3);
    }, m_elevatorSubsystem));

    // ARM:
    // rotate up
    m_driverController.povUp().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(0.2);
    }, m_armSubsystem));

    // rotate down
    m_driverController.povDown().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(-0.2);
    }, m_armSubsystem));

    // hold arm in place
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> {
      m_armSubsystem.rotateArm(0.15);
    }, m_armSubsystem));

    // grab
    m_driverController.a().whileTrue(new RunCommand(() -> {
      m_armSubsystem.grabArm(0.5);
    }, m_armSubsystem));

    // !grab
    m_driverController.b().whileTrue(new RunCommand(() -> {
      m_armSubsystem.grabArm(-0.5);
    }, m_armSubsystem));
  }

  public CommandBase getAuto() {
    DriveToCommunity m_driveToCommunity = new DriveToCommunity(m_driveTrain, true);
    DriveToChargeStaion m_driveToChargeStaion = new DriveToChargeStaion(m_driveTrain, true);
    Climb m_climb = new Climb(m_driveTrain, true);
    Stabilize m_stabilize = new Stabilize(m_driveTrain);

    return Commands.sequence(m_driveToCommunity, m_driveToChargeStaion, m_climb, m_stabilize);
  }
}
