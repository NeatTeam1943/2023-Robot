package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SensorConstants;

public class Elevator extends SubsystemBase {

  private WPI_TalonFX m_leftMotor;
  private WPI_TalonFX m_rightMotor;

  private DigitalInput m_topSwitch;
  private DigitalInput m_bottomSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    m_leftMotor = new WPI_TalonFX(ElevatorConstants.kElevatorFrontMotorID);
    m_rightMotor = new WPI_TalonFX(ElevatorConstants.kElevatorRearMotorID);

    m_bottomSwitch = new DigitalInput(SensorConstants.kBottomLimitSwitchPort);
    m_topSwitch = new DigitalInput(SensorConstants.kTopLimitSwitchPort);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    
    this.setDefaultCommand(new RunCommand(() -> {moveElevator(0);}, this));
  }

  public void moveElevator(double value) {
    // if ((m_topSwitch.get() && value > 0) || (m_bottomSwitch.get() && value < 0)) {
    //   value = 0;
    // }
    //TODO: Need for lated

    m_leftMotor.set(value);
    m_rightMotor.set(value);
    System.out.println("working!!!: " +  value);
  }

  @Override
  public void periodic() {}
}
