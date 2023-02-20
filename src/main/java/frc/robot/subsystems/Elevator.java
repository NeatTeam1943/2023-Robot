package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private WPI_TalonSRX m_motor;

  private DigitalInput m_topSwitch;
  private DigitalInput m_botomSwitch;
  /** Creates a new Elevator. */
  public Elevator() {
    m_motor = new WPI_TalonSRX(ElevatorConstants.kElevatorMotorID);
  }
  
  public void moveElevator(double value) {
    if (m_topSwitch.get() && value > 0) {
      m_motor.set(0);
    } else if (m_botomSwitch.get() && value < 0) {
      m_motor.set(0);
    } else {
      m_motor.set(value);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
