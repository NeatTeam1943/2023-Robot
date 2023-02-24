// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  /** Creates a new AutoRoutines. */
  public AutoRoutines() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static CommandBase chargeStationRoutine(PickupItem pickupItemCommand, DropItem dropItemCommand, DriveMeters driveCommand, TurnPID m_aimAtTarget){
    return Commands.sequence(pickupItemCommand,  m_aimAtTarget, driveCommand,  dropItemCommand, driveCommand);
  }

  public static CommandBase pickupRoutine(PickupItem pickupItemCommand, DropItem dropItemCommand, DriveMeters driveCommand, TurnPID m_aimAtTarget,TurnPID m_turnPID){
    return Commands.sequence(pickupItemCommand, m_aimAtTarget, driveCommand, dropItemCommand, m_turnPID, driveCommand);
  }
}
