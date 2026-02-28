// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MidSystemConstants; // TODO: new constants 
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.util.ControlCalculations;
import frc.robot.util.ControlCalculations.LaunchCalc;
import java.util.function.Supplier;

public class MidSystem extends SubsystemBase {
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Indexer indexer;
  private final Intake intake;
  private final Drive drive;
  
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;

  private final CommandXboxController controller = new CommandXboxController(0);

  // Target State
  private Translation3d currentTarget;
  private boolean inScoringArea = false;

  public MidSystem(
      Shooter shooter,
      ShooterPitch pitch,
      Indexer indexer,
      Drive drive,
      Intake intake,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotSpeedSupplier) {
    this.shooter = shooter;
    this.pitch = pitch;
    this.indexer = indexer;
    this.drive = drive;
    this.intake = intake;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedSupplier = robotSpeedSupplier;

    updateAllianceTarget();

    // 10 Hz = 0.1 seconds
    double calcPeriodSeconds = 1.0 / MidSystemConstants.CALC_RATE_HZ;
    addPeriodic(this::updateCalculations, calcPeriodSeconds);

    // Setup button bindings once on initialization
    configureBindings();
  }

  private void updateAllianceTarget() {
    currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
          ? FieldConstants.HUB_BLUE
          : FieldConstants.HUB_RED;
  }

  private void updateCalculations() {
    Pose2d robotPose = robotPoseSupplier.get();

    // TODO: DOUBLE CHECK FOR POSITION BETTER
    double distanceMeters = ControlCalculations.getDistanceToTarget(robotPose, currentTarget).in(Units.Meters);
    inScoringArea = distanceMeters <= MidSystemConstants.MAX_SHOOT_DISTANCE_METERS;

    if (inScoringArea) {
      ChassisSpeeds robotSpeed = robotSpeedSupplier.get();
      LaunchCalc launchData = ControlCalculations.MultiShot(robotPose, robotSpeed, currentTarget, 3);
      
      double targetVelocityRPS = launchData.shooterVelocity() / MidSystemConstants.FLYWHEEL_CIRCUMFERENCE_INCHES;  // TODO: Define flywheel for proper math
      
      double targetPitchRotations = launchData.shooterPitch() / 360.0; 

      shooter.setTargetVelocity(targetVelocityRPS);
      pitch.setTargetPosition(targetPitchRotations);
    } else {
      // If outside the zone, let the subsystems idle to save power
      shooter.stop();
      pitch.setTargetPosition(0); // Or a safe stow angle
    }
  }


  private boolean isRobotAimed() {
    Pose2d pose = robotPoseSupplier.get();
    Rotation2d desiredAngle = new Rotation2d(currentTarget.getX() - pose.getX(), currentTarget.getY() - pose.getY());
    
    double errorDegrees = Math.abs(desiredAngle.minus(pose.getRotation()).getDegrees()); // TODO: DETERMINE PROPER GETROTATION
    return errorDegrees < MidSystemConstants.AIM_TOLERANCE_DEGREES;
  }


  private void configureBindings() {
    // Triggers
    Trigger shootTrigger = controller.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger intakeTrigger = controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger resetGyroTrigger = controller.b();
    Trigger lockPositionTrigger = controller.x();
    Trigger aimTrigger = controller.povRight();

    // Default Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Auto-Aim Drive (Hold POV Right to point at speaker while driving)
    Supplier<Rotation2d> targetAngleSupplier = () -> new Rotation2d(
        currentTarget.getX() - robotPoseSupplier.get().getX(),
        currentTarget.getY() - robotPoseSupplier.get().getY()
    );

    aimTrigger.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            targetAngleSupplier));

    // 3. Shoot Sequence
    // Only true if the button is pressed AND all 3 systems are ready.
    Trigger readyToFire = shootTrigger
        .and(shooter::readyToShoot)
        .and(pitch::readyToShoot)
        .and(this::isRobotAimed);

    // Run the indexer. Should auto-stop when shoot sequence fails
    readyToFire.whileTrue(
        Commands.startEnd(indexer::feed, indexer::stop, indexer)
    );

    // 4. Intake
    intakeTrigger.whileTrue(
        Commands.startEnd(intake::startIntake, intake::stopIntake, intake)
    );

    // 5. Utility
    lockPositionTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));
    resetGyroTrigger.onTrue(
        Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
            .ignoringDisable(true));
  }
}