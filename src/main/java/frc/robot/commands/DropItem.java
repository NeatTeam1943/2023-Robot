// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class DropItem extends CommandBase {
  private Arm m_arm;
  private Elevator m_elevator;
  private Timer m_timer;
  
  /** Creates a new DropItem. */
  public DropItem(Arm arm, Elevator elevator) {
    m_arm = arm;
    m_elevator = elevator;
    addRequirements(m_arm, m_elevator);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.grabArm(-0.1);
    m_timer.reset();
    m_timer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 3) {
      m_arm.grabArm(0);
      m_arm.rotateArm(-0.5);
      if (m_arm.getBotomLimitSwitchState()){
        m_elevator.moveElevator(-0.5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rotateArm(0);
    m_elevator.moveElevator(0);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getBotomLimitSwitchState() && m_elevator.getBotomLimitSwitchState();
  }
}
