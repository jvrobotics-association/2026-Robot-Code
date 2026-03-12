package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignLeftWithTrench extends Command {
  private Drive m_drive;
  private PathPlannerPath path;
  private Command pathCommand;

  /** Creates a new AutoAlignLeftNoTrench. */
  public AutoAlignLeftWithTrench(Drive drive) {
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      path = PathPlannerPath.fromPathFile("Left Center to Shoot");

      PathConstraints constraints =
          new PathConstraints(
              LinearVelocity.ofBaseUnits(2, MetersPerSecond),
              LinearAcceleration.ofBaseUnits(2.5, MetersPerSecondPerSecond),
              AngularVelocity.ofBaseUnits(540, DegreesPerSecond),
              AngularAcceleration.ofBaseUnits(720, DegreesPerSecondPerSecond));

      pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

      CommandScheduler.getInstance().schedule(pathCommand);
    } catch (Exception e) {
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
