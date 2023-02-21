// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurnPIDConstants;
import frc.robot.subsystems.DriveTrain;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnPID extends PIDCommand {

  private static SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(TurnPIDConstants.kS, TurnPIDConstants.kV, TurnPIDConstants.kA);

  /** Creates a new TurnPID. */
  public TurnPID(DriveTrain drivetrain, double angleGoal) {
    super(
        // The controller that the command will use
        new PIDController(TurnPIDConstants.kTurnP, TurnPIDConstants.kTurnI, TurnPIDConstants.kTurnD),
        // Thiss should return the measurement
        () -> drivetrain.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> drivetrain.getHeading() + angleGoal,
        // This uses the output
        output -> {
          drivetrain.arcadeDrive(0, output + m_feedforward.calculate(drivetrain.getHeading() + angleGoal), false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    getController().enableContinuousInput(-180, 180);
    // Configure additional PID options by calling `getController` here.
    getController()
    .setTolerance(TurnPIDConstants.kTurnToleranceDeg, TurnPIDConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
