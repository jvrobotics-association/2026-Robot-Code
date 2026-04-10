// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Tower;
import frc.robot.subsystems.led.LEDSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdvancedShootCommand extends Command {
  private Shooter shooter;
  private Supplier<Boolean> feedOverride;
  private ShooterPitch shooterPitch;
  private Tower tower;
  private Indexer indexer;
  private LEDSystem ledSystem;
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<Translation2d> hubTarget;
  private Supplier<Alliance> alliance;

  private double hubDistance = 0.0;
  private double calculatedPitch = 0.0;
  private double calculatedShooterRPS = 0.0;
  private boolean shooterReady = false;

  private Boolean IN_RANGE = false;
  private String IN_RANGE_STATUS = "NOT CALCULATED";

  /** Creates a new AdvancedShootCommand. */
  public AdvancedShootCommand(
      Shooter shooter,
      Supplier<Boolean> feedOverride,
      ShooterPitch shooterPitch,
      Tower tower,
      Indexer indexer,
      LEDSystem ledSystem,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> hubTarget,
      Supplier<Alliance> alliance) {
    this.shooter = shooter;
    this.feedOverride = feedOverride;
    this.shooterPitch = shooterPitch;
    this.tower = tower;
    this.indexer = indexer;
    this.ledSystem = ledSystem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.hubTarget = hubTarget;
    this.alliance = alliance;

    addRequirements(shooter, shooterPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculateShot();
    shooterPitch.aim(calculatedPitch);
    shooter.runShooter(calculatedShooterRPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculateShot();
    shooterPitch.aim(calculatedPitch);
    shooter.runShooter(calculatedShooterRPS);

    if (shooterReady && isFacingTarget(robotPoseSupplier.get(), hubTarget.get())
        || (calculatedShooterRPS >= 0 && feedOverride.get())) {
      indexer.feed();
      tower.start();
    } else {
      shooterReady = shooter.atTargetVelocity() && shooterPitch.isAtPosition();
      indexer.stop();
      tower.stop();
    }

    Logger.recordOutput("AdvancedShootCommand/shooterReady", shooterReady);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    tower.stop();
    shooter.stop();
    shooterPitch.stop();
    shooterReady = false;
  }

  private void calculateShot() {
    hubDistance =
        Inches.convertFrom(
            robotPoseSupplier.get().getTranslation().getDistance(hubTarget.get()), Meters);

    if (hubDistance < 75.6) {
      calculatedPitch = 0;
      calculatedShooterRPS = 0;

      if (alliance.get() == Alliance.Blue) {
        // ledSystem.setEntireLeftSide(AnimationType.FlowDirectionBlueInverted);
        // ledSystem.setEntireRightSide(AnimationType.FlowDirectionBlue);
      } else {
        // ledSystem.setEntireLeftSide(AnimationType.FlowDirectionRedInverted);
        // ledSystem.setEntireRightSide(AnimationType.FlowDirectionRed);
      }

      IN_RANGE_STATUS = "TOO CLOSE";
      IN_RANGE = false;

    } else if (hubDistance >= 75.6 && hubDistance < 95.1) {
      calculatedPitch = ShooterConstants.CLOSE_PITCH;
      calculatedShooterRPS = ShooterConstants.CLOSE_SHOT.get(hubDistance);
      // ledSystem.setAll(AnimationType.SolidGreen);
      IN_RANGE_STATUS = "NEAR SHOT";
      IN_RANGE = true;
    } else if (hubDistance >= 95.1 && hubDistance <= 118.9) {
      calculatedPitch = ShooterConstants.FAR_PITCH;
      calculatedShooterRPS = ShooterConstants.FAR_SHOT.get(hubDistance);
      // ledSystem.setAll(AnimationType.StrobeGreen);
      IN_RANGE_STATUS = "FAR SHOT";
      IN_RANGE = true;
    } else if (hubDistance > 118.9) {
      calculatedPitch = 0;
      calculatedShooterRPS = 0;

      if (alliance.get() == Alliance.Blue) {
        // ledSystem.setEntireLeftSide(AnimationType.FlowDirectionBlue);
        // ledSystem.setEntireRightSide(AnimationType.FlowDirectionBlueInverted);
      } else {
        // ledSystem.setEntireLeftSide(AnimationType.FlowDirectionRed);
        // ledSystem.setEntireRightSide(AnimationType.FlowDirectionRedInverted);
      }
      IN_RANGE_STATUS = "TOO FAR";
      IN_RANGE = false;
    }

    Logger.recordOutput("AdvancedShootCommand/hubDistance", hubDistance);
    Logger.recordOutput("AdvancedShootCommand/calculatedPitch", calculatedPitch);
    Logger.recordOutput("AdvancedShootCommand/calculatedShooterRPS", calculatedShooterRPS);

    Logger.recordOutput("AdvancedShootCommand/IN_RANGE", IN_RANGE);
    Logger.recordOutput("AdvancedShootCommand/IN_RANGE_STATUS", IN_RANGE_STATUS);
  }

  @AutoLogOutput(key = "AdvancedShootCommand/isFacingTarget")
  public boolean isFacingTarget(Pose2d pose, Translation2d target) {
    Rotation2d error =
        target
            .minus(pose.getTranslation())
            .getAngle()
            .minus(pose.getRotation().plus(Rotation2d.fromDegrees(180)));

    boolean facingTarget = Math.abs(error.getDegrees()) <= 2.0;
    Logger.recordOutput("AdvancedShootCommand/isFacingTarget", facingTarget);
    return facingTarget;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
