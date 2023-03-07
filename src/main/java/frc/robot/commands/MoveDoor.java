// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Door;

public class MoveDoor extends CommandBase {
  private Door m_door;
  private double m_value;
  private boolean m_Opening;
  /** Creates a new MoveDoor. */
  public MoveDoor(Door door, double value) {
    m_door = door;
    m_value = value;
    addRequirements(m_door);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_door.moveDoor(m_value);
    if (m_value>0){
      m_Opening = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Opening){
      return m_door.getCloseSwitch();
    } else {
      return m_door.getOpenSwitch();
    }
  }
}
