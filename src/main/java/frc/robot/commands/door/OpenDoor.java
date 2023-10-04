// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.door;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DoorConstants;
import frc.robot.subsystems.Door;

public class OpenDoor extends CommandBase {
  private Door m_door;

  public OpenDoor(Door door) {
    m_door = door;

    addRequirements(m_door);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========== Start OpenDoor() ==========");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_door.move(-DoorConstants.kDoorSpeed);

    System.out.println("Switch Open pressed? " + m_door.isOpen());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_door.move(0);
    System.out.println("========== Finished OpenDoor() ==========");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_door.isOpen();
  }
}
