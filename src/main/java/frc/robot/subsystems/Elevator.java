package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private WPI_TalonFX m_leftMotor;
  private WPI_TalonFX m_rightMotor;

  private DigitalInput m_topSwitch;
  private DigitalInput m_botomSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    m_leftMotor = new WPI_TalonFX(ElevatorConstants.kElevatorFrontMotorID);
    m_rightMotor = new WPI_TalonFX(ElevatorConstants.kElevatorRearMotorID);
  }

  public void moveElevator(double value) {
    if ((m_topSwitch.get() && value > 0) || (m_botomSwitch.get() && value < 0)) {
      value = 0;
    }

    m_leftMotor.set(value);
    m_rightMotor.set(value);
  }

  @Override
  public void periodic() {
    this.setDefaultCommand(new RunCommand(() -> {moveElevator(0);}, this));
  }
}
