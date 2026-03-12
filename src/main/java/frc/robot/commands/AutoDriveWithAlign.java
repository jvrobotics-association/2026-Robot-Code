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
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveWithAlign extends Command {
  private Drive drive;
  private PathPlannerPath leftShootFromNeutralZonePath;
  private PathPlannerPath rightShootFromNeutralZonePath;
  private Command pathCommand;
  private Pose2d robotPose;

  /** Creates a new AutoAlignLeftNoTrench. */
  public AutoDriveWithAlign(Drive drive) {
    this.drive = drive;
    robotPose = drive.getPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      leftShootFromNeutralZonePath = PathPlannerPath.fromPathFile("LS-NZ");
      rightShootFromNeutralZonePath = PathPlannerPath.fromPathFile("RS-NZ");

      PathConstraints constraints =
          new PathConstraints(
              LinearVelocity.ofBaseUnits(3.5, MetersPerSecond),
              LinearAcceleration.ofBaseUnits(4.9, MetersPerSecondPerSecond),
              AngularVelocity.ofBaseUnits(540, DegreesPerSecond),
              AngularAcceleration.ofBaseUnits(720, DegreesPerSecondPerSecond));

      boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

      robotPose = drive.getPose();
      Distance xPos = robotPose.getMeasureX();
      Distance yPos = robotPose.getMeasureY();

      boolean isNeutralZone =
          isBlue
                  && xPos.gte(Distance.ofRelativeUnits(182.11, Inch))
                  && xPos.lte(Distance.ofRelativeUnits(469.11, Inch))
              || !isBlue
                  && xPos.gte(Distance.ofRelativeUnits(182.11, Inch))
                  && xPos.lte(Distance.ofRelativeUnits(469.11, Inch));

      boolean isLeft =
          isBlue && yPos.gte(Distance.ofRelativeUnits(158.84, Inch))
              || !isBlue && yPos.lte(Distance.ofRelativeUnits(158.84, Inch));

      // Logger.recordOutput("AutoDriveWithAlign/isNeutralZone", isNeutralZone);
      // Logger.recordOutput("AutoDriveWithAlign/isBlue", isBlue);
      // Logger.recordOutput("AutoDriveWithAlign/isLeft", isLeft);

      if (isNeutralZone && isLeft) {
        pathCommand = AutoBuilder.pathfindThenFollowPath(leftShootFromNeutralZonePath, constraints);
      } else if (isBlue && !isNeutralZone && isLeft) {
        pathCommand = AutoBuilder.pathfindToPose(FieldConstants.LEFT_SHOOT_POS_BLUE, constraints);
      } else if (!isBlue && !isNeutralZone && isLeft) {
        pathCommand = AutoBuilder.pathfindToPose(FieldConstants.LEFT_SHOOT_POS_RED, constraints);
      } else if (isNeutralZone && !isLeft) {
        pathCommand =
            AutoBuilder.pathfindThenFollowPath(rightShootFromNeutralZonePath, constraints);
      } else if (isBlue && !isNeutralZone && !isLeft) {
        pathCommand = AutoBuilder.pathfindToPose(FieldConstants.RIGHT_SHOOT_POS_BLUE, constraints);
      } else if (!isBlue && !isNeutralZone && !isLeft) {
        pathCommand = AutoBuilder.pathfindToPose(FieldConstants.RIGHT_SHOOT_POS_RED, constraints);
      }

      CommandScheduler.getInstance().schedule(pathCommand);
    } catch (Exception e) {
      Logger.recordOutput("AutoDriveWithAlign", e.getStackTrace().toString());
      System.out.println("Unable to run auto");
      e.printStackTrace();
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
