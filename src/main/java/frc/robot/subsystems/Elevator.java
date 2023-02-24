package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private WPI_TalonSRX m_motor;

  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_botomLimitSwitch;
  /** Creates a new Elevator. */
  public Elevator() {
    m_motor = new WPI_TalonSRX(ElevatorConstants.kElevatorMotor);
  }
  
  public void moveElevator(double value) {
    if (m_topLimitSwitch.get() && value > 0) {
      m_motor.set(0);
    } else if (m_botomLimitSwitch.get() && value < 0) {
      m_motor.set(0);
    } else {
      m_motor.set(value);
    }
  }

  public boolean getTopLimitSwitchState(){
    return m_topLimitSwitch.get();
  }

  public boolean getBotomLimitSwitchState(){
    return m_botomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
