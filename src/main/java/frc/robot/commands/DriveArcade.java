package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class DriveArcade extends CommandBase {
  private DriveTrain m_driveTrain;
  private CommandXboxController m_joystick;

  public DriveArcade(DriveTrain driveTrain, CommandXboxController joystick) {
    m_driveTrain = driveTrain;
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
    double mov = m_joystick.getLeftY();
    double rot = m_joystick.getLeftX();

    m_driveTrain.arcadeDrive(-mov, rot);
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
