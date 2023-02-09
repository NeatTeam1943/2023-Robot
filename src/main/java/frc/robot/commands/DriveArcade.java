package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SimDrive;

public class DriveArcade extends CommandBase {
  private SimDrive m_driveTrain;
  private XboxController m_joystick;

  public DriveArcade(SimDrive m_simDrive, XboxController joystick) {
    m_driveTrain = m_simDrive;
    m_joystick = joystick;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mov = m_joystick.getLeftX();
    double rot = m_joystick.getLeftY();
    m_driveTrain.robotArcadeDrive(-mov, -rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
