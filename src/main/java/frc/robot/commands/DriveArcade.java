package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SimDrive;

public class DriveArcade extends CommandBase {
  private DriveTrain m_driveTrain;
  private SimDrive m_driveTrainSim;
  private CommandXboxController m_joystick;
  private boolean isSim;

  public DriveArcade(DriveTrain driveTrain, CommandXboxController joystick) {
    m_driveTrain = driveTrain;
    m_joystick = joystick;
    this.isSim = false; 
    addRequirements(m_driveTrain);
  }

  public DriveArcade(SimDrive driveTrain, CommandXboxController joystick) {
    m_driveTrainSim = driveTrain;
    m_joystick = joystick;
    this.isSim = true; 
    addRequirements(m_driveTrainSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.isSim){
      double mov = m_joystick.getLeftX();
      double rot = m_joystick.getLeftY();
      m_driveTrainSim.robotArcadeDrive(-mov, -rot);
    }
    else{
      double mov = m_joystick.getLeftX();
      double rot = m_joystick.getLeftY();
      m_driveTrain.arcadeDrive(-mov, -rot);
    }

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
