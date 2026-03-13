// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveUnderTrench extends Command {
  private Drive drive;
  private PathPlannerPath leftTrenchTranversalPath;
  private PathPlannerPath rightTrenchTranversalPath;
  private Command pathCommand;
  private Pose2d robotPose;

  public AutoDriveUnderTrench(Drive drive) {
    this.drive = drive;
    robotPose = drive.getPose();
  }

  @Override
  public void initialize() {
    try {
      leftTrenchTranversalPath = PathPlannerPath.fromPathFile("LA-NZ");
      rightTrenchTranversalPath = PathPlannerPath.fromPathFile("RA-NZ");

      PathConstraints constraints =
          new PathConstraints(
              LinearVelocity.ofBaseUnits(3.5, MetersPerSecond),
              LinearAcceleration.ofBaseUnits(6, MetersPerSecondPerSecond),
              AngularVelocity.ofBaseUnits(540, DegreesPerSecond),
              AngularAcceleration.ofBaseUnits(720, DegreesPerSecondPerSecond));

      boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

      robotPose = drive.getPose();
      Distance yPos = robotPose.getMeasureY();

      boolean isLeft =
          isBlue && yPos.gte(Distance.ofRelativeUnits(158.84, Inch))
              || !isBlue && yPos.lte(Distance.ofRelativeUnits(158.84, Inch));

      Logger.recordOutput("AutoDriveUnderTrench/isBlue", isBlue);
      Logger.recordOutput("AutoDriveUnderTrench/isLeft", isLeft);

      if (isLeft) {
        pathCommand = AutoBuilder.pathfindThenFollowPath(leftTrenchTranversalPath, constraints);
      } else {
        pathCommand = AutoBuilder.pathfindThenFollowPath(rightTrenchTranversalPath, constraints);
      }

      CommandScheduler.getInstance().schedule(pathCommand);
    } catch (Exception e) {
      Logger.recordOutput("AutoDriveUnderTrench", e.getStackTrace().toString());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Optional.ofNullable(pathCommand).ifPresent(Command::cancel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
