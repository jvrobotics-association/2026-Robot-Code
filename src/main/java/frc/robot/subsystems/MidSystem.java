// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MidSystemConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPitchConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
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
  private final IntakeExtension intakeExt;
  private final Drive drive;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;

  private final CommandXboxController controller = new CommandXboxController(0);

  // Target State
  private Translation3d currentTarget;

  public MidSystem(
      Shooter shooter, 
      ShooterPitch pitch, 
      Indexer indexer, 
      Drive drive, 
      Intake intake,
      IntakeExtension intakeExt) {
    
    this.shooter = shooter;
    this.pitch = pitch;
    this.indexer = indexer;
    this.drive = drive;
    this.intake = intake;
    this.intakeExt = intakeExt;
    
    this.robotPoseSupplier = drive::getPose;
    this.robotSpeedSupplier = drive::getChassisSpeeds;

    updateAllianceTarget();
    configureBindings();
  }

  private void updateAllianceTarget() {
    currentTarget =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;
  }

  private boolean inScoringArea() {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue
            && robotPoseSupplier.get().getMeasureX().lt(FieldConstants.ALLIANCE_ZONE.plus(RobotConstants.FULL_WIDTH.div(2)))
        || !isBlue
            && robotPoseSupplier.get().getMeasureX().gt(FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE.plus(RobotConstants.FULL_WIDTH.div(2))));
  }

  /**
   * Maps the calculated shot angle to physical motor rotations.
   * Handles inverted relationships and clamps the output to physical limits.
   */
  private double calculatePitchRotations(double targetAngleDegrees) {
    if (ShooterPitchConstants.MAX_SHOT_ANGLE == ShooterPitchConstants.MIN_SHOT_ANGLE) return 0; 

    double slope = (ShooterPitchConstants.MIN_ROTATION - ShooterPitchConstants.MAX_ROTATION) 
                 / (ShooterPitchConstants.MAX_SHOT_ANGLE - ShooterPitchConstants.MIN_SHOT_ANGLE);
                 
    double rawRotations = ShooterPitchConstants.MAX_ROTATION + slope * (targetAngleDegrees - ShooterPitchConstants.MIN_SHOT_ANGLE);

    return MathUtil.clamp(rawRotations, ShooterPitchConstants.MIN_ROTATION, ShooterPitchConstants.MAX_ROTATION);
  }

  @Override
  public void periodic() {
    if (inScoringArea()) {
      LaunchCalc launchData = ControlCalculations.MultiShot(
              robotPoseSupplier.get(), robotSpeedSupplier.get(), currentTarget, 3);

      double targetVelocityRPS = launchData.shooterVelocity() / ShooterConstants.FLYWHEEL_CIRCUMFERENCE;
      double targetPitchRotations = calculatePitchRotations(launchData.shooterPitch());

      shooter.setTargetVelocity(targetVelocityRPS);
      pitch.setTargetPosition(targetPitchRotations);
    } else {
      shooter.stop();
      pitch.setTargetPosition(0); 
    }
  }

  private boolean isRobotAimed() {
    Pose2d pose = robotPoseSupplier.get();
    Rotation2d desiredAngle = new Rotation2d(currentTarget.getX() - pose.getX(), currentTarget.getY() - pose.getY());

    double errorDegrees = Math.abs(desiredAngle.minus(pose.getRotation()).getDegrees()); 
    return errorDegrees < MidSystemConstants.AIM_TOLERANCE_DEGREES;
  }

  private void configureBindings() {
    // Controller Inputs
    Trigger shootTrigger = controller.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger intakeTrigger = controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger resetGyroTrigger = controller.b();
    Trigger lockPositionTrigger = controller.x();
    Trigger aimTrigger = controller.povRight();
    Trigger climbTrigger = controller.y(); 

    // Default Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Auto-Aim Drive
    Supplier<Rotation2d> targetAngleSupplier = () -> new Rotation2d(
                currentTarget.getX() - robotPoseSupplier.get().getX(),
                currentTarget.getY() - robotPoseSupplier.get().getY());

    aimTrigger.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            targetAngleSupplier));

    // Shoot Sequence (Indexer logic)
    Trigger readyToFire = shootTrigger.and(shooter::readyToShoot).and(pitch::readyToShoot).and(this::isRobotAimed);
    readyToFire.whileTrue(Commands.startEnd(indexer::feed, indexer::stop, indexer));

    // Intake Rollers
    intakeTrigger.whileTrue(Commands.startEnd(intake::startIntake, intake::stopIntake, intake));

    // Intake Extension Logic
    // Automatically deploy when the robot is enabled
    Trigger isEnabled = new Trigger(DriverStation::isEnabled);
    isEnabled.onTrue(Commands.runOnce(intakeExt::deploy, intakeExt));

    // Retract when the driver prepares to climb
    climbTrigger.onTrue(Commands.runOnce(intakeExt::retract, intakeExt));

    // Utility
    lockPositionTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));
    resetGyroTrigger.onTrue(
        Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
            .ignoringDisable(true));
  }
}