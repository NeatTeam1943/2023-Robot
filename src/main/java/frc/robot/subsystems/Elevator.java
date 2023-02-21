package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private WPI_TalonSRX m_leftMotor;
  private WPI_TalonSRX m_rightMotor;

  private DigitalInput m_topSwitch;
  private DigitalInput m_botomSwitch;
  /** Creates a new Elevator. */
  public Elevator() {
    m_leftMotor = new WPI_TalonSRX(ElevatorConstants.kElevatorLeftMotorID);
    m_rightMotor = new WPI_TalonSRX(ElevatorConstants.kElevatorRightMotorID);

  }
  
  public void moveElevator(double value) {
    if (m_topSwitch.get() && value > 0) {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    } else if (m_botomSwitch.get() && value < 0) {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    } else {
      m_leftMotor.set(value);
      m_rightMotor.set(-value);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
