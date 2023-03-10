// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CommunityConstants;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveToChargeStaion;
import frc.robot.commands.auto.PassChargeStation;
import frc.robot.commands.auto.Stabilize;
import frc.robot.commands.door.CloseDoor;
import frc.robot.commands.door.OpenDoor;
import frc.robot.subsystems.Door;
import frc.robot.subsystems.DriveTrain;

public class AutoFunctions {
  private DriveTrain m_drive;
  private Door m_door;

  public AutoFunctions(DriveTrain drive, Door door) {
    m_drive = drive;
    m_door = door;
  }

  /**
   * Exit community being close to the line
   *
   * <p>Requirements: Placing the robot right on the line facing the grids
   */
  public Command passLine() {
    return new DriveDistance(m_drive, CommunityConstants.kLineDistance, true);
  }

  /**
   * Exit community from grids on the short side
   *
   * <p>Requirements: Placing the robot next to the grids and opposing alliance loading zone facing
   * the grids
   */
  public Command passShort() {
    return new DriveDistance(m_drive, CommunityConstants.kShortDistance, true);
  }

  /**
   * Exit community from grids on the long side
   *
   * <p>Requirements: Placing the robot next to the grids facing the grids
   */
  public Command passLong() {
    return new DriveDistance(m_drive, CommunityConstants.kLongDistance, true);
  }

  /**
   * Exit the comunity through the charge station
   *
   * <p>Requirements: Placing the robot in front of the charge station facing the grids
   */
  public Command passChargeStation() {
    return new PassChargeStation(m_drive, true);
  }

  /**
   * Driving to the charge station and stabilizing
   *
   * <p>Requirements: Placing the robot in front of the charge station facing the grids
   *
   * @param backwards reverse the robot
   */
  public Command stabilize(boolean backwards) {
    Command driveAndClose =
        Commands.parallel(new DriveToChargeStaion(m_drive, backwards), new CloseDoor(m_door));

    return Commands.sequence(
        driveAndClose, new DriveDistance(m_drive, 0.93, backwards), new Stabilize(m_drive));
  }

  /**
   * Exit the community through the charge station and then drive back to the charge station and
   * stabilize
   *
   * <p>Requirements: Placing the robot in front of the charge station facing the grids
   */
  public Command passChargeStationAndStabilize() {
    return Commands.sequence(passChargeStation(), stabilize(false));
  }

  /**
   * Score one game piece
   *
   * <p>Requirements: Placing the robot close to the grids facing the grids
   */
  public Command gamePiece() {
    Command closeAndDriveDistance =
        Commands.parallel(new DriveDistance(m_drive, 0.3, true), new CloseDoor(m_door));

    return Commands.sequence(new OpenDoor(m_door), closeAndDriveDistance);
  }

  /**
   * Score one game piece, then drive to charge station and stabilize
   *
   * <p>Requirements: Place the robot close the grids in front of the charge station facing the
   * grids
   */
  public Command gamePieceAndStabilize() {
    return Commands.sequence(new OpenDoor(m_door), stabilize(true));
  }

  /**
   * Score one game piece and leave the community on the short side
   *
   * <p>Requirements: Placing the robot close to the grids and opposing allience loading zone facing
   * the grids
   */
  public Command gamePieceAndPassShort() {
    Command closeAndPassShort = Commands.parallel(new CloseDoor(m_door), passShort());

    return Commands.sequence(new OpenDoor(m_door), closeAndPassShort);
  }

  /**
   * Score one game piece and leave the communtiy on the long side
   *
   * <p>Requirements: Place the robot close to the grids and the barier facing the grids
   */
  public Command gamePieceAndPassLong() {
    Command closeAndPassLong = Commands.parallel(new CloseDoor(m_door), passLong());

    return Commands.sequence(new OpenDoor(m_door), closeAndPassLong);
  }

  /**
   * Score one game piece, leave the commuity trough the charge station, go back to charge station
   * and stabilize
   *
   * <p>Requirements: Place the robot close the grids in front of the charge station facing the
   * grids
   */
  public Command fullRoute() {
    Command closeAndPassChargeStation =
        Commands.parallel(new CloseDoor(m_door), passChargeStation());

    return Commands.sequence(new OpenDoor(m_door), closeAndPassChargeStation, stabilize(false));
  }
}
