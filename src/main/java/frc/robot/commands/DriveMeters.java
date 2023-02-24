// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveMeters extends PIDCommand {
  private DriveTrain m_driveTrain;
  private static SimpleMotorFeedforward m_feedforward;

  /** Creates a new Drive. */
  public DriveMeters(DriveTrain driveTrain, double distanceToDrive) {
    super(
        // The controller that the command will use
        new PIDController(PIDConstants.kForwardP, PIDConstants.kForwardI, PIDConstants.kForwardD),
        // This should return the measurement
        driveTrain::getDistance,
        // This should return the setpoint (can also be a constant)
        () -> driveTrain.getDistance() + distanceToDrive,
        // This uses the output
        output -> {
          // Use the output here
          driveTrain.arcadeDrive(output + m_feedforward.calculate(driveTrain.getDistance() + distanceToDrive), 0, true);
        });
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);

    getController().setTolerance(0.3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
