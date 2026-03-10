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
import frc.robot.Constants.ShooterPitchConstants;
import frc.robot.commands.AutoAlignLeftWithTrench;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Tower;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MidSystem extends SubsystemBase {
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Indexer indexer;
  private final Tower tower;
  private final Intake intake;
  private final IntakeExtension intakeExt;
  private final Hopper hopper;
  private final Climber climber;
  private final Drive drive;
  private final Vision vision;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;

  private final CommandXboxController controller = new CommandXboxController(0);

  // Target State
  private Translation3d currentTarget;
  private double currentTowerRPS = 0;
  private boolean isClimbPrepared = false;

  public MidSystem(
      Shooter shooter,
      ShooterPitch pitch,
      Indexer indexer,
      Tower tower,
      Drive drive,
      Vision vision,
      Intake intake,
      IntakeExtension intakeExt,
      Hopper hopper,
      Climber climber) {

    this.shooter = shooter;
    this.pitch = pitch;
    this.indexer = indexer;
    this.tower = tower;
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.intakeExt = intakeExt;
    this.hopper = hopper;
    this.climber = climber;

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
            && robotPoseSupplier
                .get()
                .getMeasureX()
                .lt(FieldConstants.ALLIANCE_ZONE.plus(RobotConstants.FULL_WIDTH.div(2)))
        || !isBlue
            && robotPoseSupplier
                .get()
                .getMeasureX()
                .gt(
                    FieldConstants.FIELD_LENGTH.minus(
                        FieldConstants.ALLIANCE_ZONE.plus(RobotConstants.FULL_WIDTH.div(2))));
  }

  private double calculatePitchRotations(double targetAngleDegrees) {
    // if (ShooterPitchConstants.MAX_SHOT_ANGLE == ShooterPitchConstants.MIN_SHOT_ANGLE) {
    //     return 0;
    // }
    double slope =
        (ShooterPitchConstants.MIN_ROTATION - ShooterPitchConstants.MAX_ROTATION)
            / (ShooterPitchConstants.MAX_SHOT_ANGLE - ShooterPitchConstants.MIN_SHOT_ANGLE);

    Logger.recordOutput("Control Calcs/Slope", slope);

    double rawRotations =
        ShooterPitchConstants.MAX_ROTATION
            + slope * (targetAngleDegrees - ShooterPitchConstants.MIN_SHOT_ANGLE);

    Logger.recordOutput("Control Calcs/rawRotations", rawRotations);

    double distance =
        MathUtil.clamp(
            rawRotations, ShooterPitchConstants.MIN_ROTATION, ShooterPitchConstants.MAX_ROTATION);

    Logger.recordOutput("Control Calcs/distance", distance);

    return distance;
  }

  // public boolean isHubActive() {
  //   Optional<Alliance> alliance = DriverStation.getAlliance();
  //   // If we have no alliance, we cannot be enabled, therefore no hub.
  //   if (alliance.isEmpty()) {
  //     return false;
  //   }
  //   // Hub is always enabled in autonomous.
  //   if (DriverStation.isAutonomousEnabled()) {
  //     return true;
  //   }
  //   // At this point, if we're not teleop enabled, there is no hub.
  //   if (!DriverStation.isTeleopEnabled()) {
  //     return false;
  //   }

  //   // We're teleop enabled, compute.
  //   double matchTime = DriverStation.getMatchTime();
  //   String gameData = DriverStation.getGameSpecificMessage();
  //   // If we have no game data, we cannot compute, assume hub is active, as its likely early in
  //   // teleop.
  //   if (gameData.isEmpty()) {
  //     return true;
  //   }

  //   boolean redInactiveFirst = false;
  //   switch (gameData.charAt(0)) {
  //     case 'R' -> redInactiveFirst = true;
  //     case 'B' -> redInactiveFirst = false;
  //     default -> {
  //       // If we have invalid game data, assume hub is active.
  //       return true;
  //     }
  //   }

  //   // Shift was is active for blue if red won auto, or red if blue won auto.
  //   boolean shift1Active =
  //       switch (alliance.get()) {
  //         case Red -> !redInactiveFirst;
  //         case Blue -> redInactiveFirst;
  //       };

  //   if (matchTime > 130) {
  //     // Transition shift, hub is active.
  //     return true;
  //   } else if (matchTime > 105) {
  //     // Shift 1
  //     return shift1Active;
  //   } else if (matchTime > 80) {
  //     // Shift 2
  //     return !shift1Active;
  //   } else if (matchTime > 55) {
  //     // Shift 3
  //     return shift1Active;
  //   } else if (matchTime > 30) {
  //     // Shift 4
  //     return !shift1Active;
  //   } else {
  //     // End game, hub always active.
  //     return true;
  //   }
  // }

  @Override
  public void periodic() {
    // Auto-Spooling when in scoring area
    // if (inScoringArea() && !isClimbPrepared) {
    //   LaunchCalc launchData =
    //       ControlCalculations.MultiShot(
    //           robotPoseSupplier.get(), robotSpeedSupplier.get(), currentTarget, 3);

    //   double targetVelocityRPS =
    //       launchData.shooterVelocity() / ShooterConstants.FLYWHEEL_CIRCUMFERENCE;
    //   double targetPitchRotations = calculatePitchRotations(launchData.shooterPitch());

    //   shooter.setTargetVelocity(targetVelocityRPS);
    //   pitch.setTargetPosition(targetPitchRotations);

    //   // Calculate Tower speed (50% less than shooter)
    //   currentTowerRPS = targetVelocityRPS * 0.5;
    // } else {
    //   shooter.stop();
    //   pitch.setTargetPosition(0);
    //   currentTowerRPS = 0;
    // }
    // LaunchCalc launchData =
    //     ControlCalculations.MultiShot(
    //         robotPoseSupplier.get(), robotSpeedSupplier.get(), currentTarget, 3);

    // double targetVelocityRPS =
    //     launchData.shooterVelocity() / ShooterConstants.FLYWHEEL_CIRCUMFERENCE;
    // double targetPitchRotations = calculatePitchRotations(launchData.shooterPitch());

    // shooter.setTargetVelocity(targetVelocityRPS);
    // pitch.setTargetPosition(targetPitchRotations);
  }

  private boolean isRobotAimed() {
    Pose2d pose = robotPoseSupplier.get();
    Rotation2d desiredAngle =
        new Rotation2d(currentTarget.getX() - pose.getX(), currentTarget.getY() - pose.getY());
    double errorDegrees = Math.abs(desiredAngle.minus(pose.getRotation()).getDegrees());
    return errorDegrees < MidSystemConstants.AIM_TOLERANCE_DEGREES;
  }

  private void configureBindings() {
    // Controller Inputs
    Trigger shootTrigger = controller.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger intakeTrigger = controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD);
    Trigger resetGyroTrigger = controller.start();
    // Trigger lockPositionTrigger = controller.x();
    Trigger aimTrigger = controller.leftBumper();
    Trigger climbTrigger = controller.b();
    Trigger hopperManualInTrigger = controller.povDown();
    Trigger hopperOverrideInTrigger = controller.button(7);
    Trigger hopperManualOutTrigger = controller.povUp();
    Trigger intakeManualOuttake = controller.povLeft();
    Trigger raiseArm = controller.rightBumper();
    Trigger followPathToShoot = controller.x();

    Supplier<Rotation2d> targetAngleSupplier =
        () ->
            new Rotation2d(
                currentTarget.getX() - robotPoseSupplier.get().getX(),
                currentTarget.getY() - robotPoseSupplier.get().getY());

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    followPathToShoot.whileTrue(new AutoAlignLeftWithTrench(drive));

    hopperOverrideInTrigger.whileTrue(
        Commands.runEnd(() -> hopper.setManualDutyCycle(-0.1), hopper::stop, hopper));

    hopperManualOutTrigger.onTrue(
        Commands.sequence(
            Commands.runOnce(hopper::deploy, hopper),
            Commands.waitSeconds(1),
            Commands.runOnce(intakeExt::deploy, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(intakeExt::stop, intakeExt)));

    hopperManualInTrigger.onTrue(
        Commands.sequence(
            Commands.runOnce(intakeExt::retract, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(intakeExt::stop, intakeExt),
            Commands.runOnce(hopper::retract, hopper),
            Commands.waitSeconds(5),
            Commands.runOnce(hopper::stop, hopper)));

    intakeTrigger.whileTrue(Commands.startEnd(intake::startIntake, intake::stopIntake, intake));

    intakeManualOuttake.whileTrue(
        Commands.parallel(
            Commands.startEnd(() -> intake.setManualDutyCycle(-0.5), intake::stopIntake, intake),
            Commands.startEnd(() -> indexer.setManualDutyCycle(-20), indexer::stop, indexer)));

    aimTrigger.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            targetAngleSupplier));

    // shootTrigger.whileTrue(
    //     Commands.sequence(
    //             Commands.parallel(
    //                 Commands.run(pitch::moveToPosition, pitch),
    //                 Commands.runOnce(shooter::shoot, shooter)),
    //             Commands.waitSeconds(2),
    //             Commands.parallel(
    //                 Commands.run(pitch::moveToPosition, pitch),
    //                 Commands.startEnd(indexer::feed, indexer::stop, indexer),
    //                 Commands.runEnd(() -> tower.setManualDutyCycle(0.8), tower::stop, tower)))
    //         .finallyDo(
    //             () -> {
    //               pitch.movetoMinPosition();
    //               shooter.stop();
    //             }));

    shootTrigger.whileTrue(
        Commands.parallel(
                Commands.run(pitch::moveToPosition, pitch),
                Commands.run(shooter::shoot, shooter),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.parallel(
                        Commands.startEnd(indexer::feed, indexer::stop, indexer),
                        Commands.runEnd(() -> tower.setManualDutyCycle(0.8), tower::stop, tower))))
            .finallyDo(
                () -> {
                  pitch.movetoMinPosition();
                  shooter.stop();
                }));

    raiseArm.onTrue(
        Commands.sequence(
                Commands.runOnce(intakeExt::retract, intakeExt),
                Commands.runOnce(intake::startIntake, intake),
                Commands.waitSeconds(1),
                Commands.runOnce(intakeExt::deploy, intakeExt))
            .finallyDo(() -> intake.stopIntake()));

    climbTrigger.onTrue(
        Commands.either(
            Commands.sequence(
                Commands.runOnce(climber::climb, climber),
                Commands.runOnce(() -> isClimbPrepared = false)),
            Commands.sequence(
                Commands.runOnce(intakeExt::retract, intakeExt),
                Commands.waitSeconds(1),
                Commands.runOnce(hopper::retract, hopper),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> isClimbPrepared = true)),
            () -> isClimbPrepared));

    // util
    // lockPositionTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));

    resetGyroTrigger.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    // Trigger isEnabled = new Trigger(DriverStation::isEnabled);
    // isEnabled.onTrue(
    //     Commands.sequence(
    //         Commands.runOnce(hopper::deploy, hopper),
    //         Commands.waitSeconds(1),
    //         Commands.runOnce(intakeExt::deploy, intakeExt),
    //         Commands.waitSeconds(1),
    //         Commands.runOnce(hopper::stop, hopper),
    //         Commands.runOnce(intakeExt::stop, intakeExt)));
  }
}
