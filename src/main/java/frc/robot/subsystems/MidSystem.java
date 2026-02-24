// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.util.ControlCalculations;
import frc.robot.util.ControlCalculations.LaunchCalc;

public class MidSystem extends SubsystemBase {
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;
  private final Intake intake;
  private final Drive drive;

  //Calls for the Alliance Color from Driver Station. If it can't get the Alliance it defaults to Blue
  Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? FieldConstants.HUB_BLUE : FieldConstants.HUB_RED;
  
  //Controllers
  private final CommandXboxController controller = new CommandXboxController(0);


  //Triggers
  private final Trigger shootTrigger =
      controller.rightTrigger(ControllerConstants.TRIGGER_THRESHOLD);
  private final Trigger intakeTrigger =
      controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD);
  private final Trigger resetGyroTrigger = controller.b();
  private final Trigger lock0DriveTrigger = controller.a();
  private final Trigger lockPositionTrigger = controller.x();
  private final Trigger climbTrigger = controller.y();
  private final Trigger aimTrigger = controller.povRight();

  // Shooter Variables
  private boolean isShooting = false;
  private double hoodAngle = 0;
  private double exitVelocity = 0;
  private LaunchCalc launchData;
  private ChassisSpeeds robotSpeed;
  private Pose2d robotPose;

  /** Creates a new MidSystems. */
  public MidSystem(Shooter shooter, ShooterPitch pitch, Drive drive, Intake intake, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier) {
    this.shooter = shooter;
    this.pitch = pitch;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedSupplier = robotSpeedSupplier;
    this.drive = drive;
    this.intake = intake;

  }

  @Override
  public void periodic() {
    if(isShooting){   
      robotSpeed = robotSpeedSupplier.get();
      robotPose = robotPoseSupplier.get();
      launchData = ControlCalculations.MultiShot(robotPose, robotSpeed, currentTarget, 3);
      hoodAngle = launchData.shooterPitch();
      exitVelocity = launchData.shooterVelocity();
    }

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    lock0DriveTrigger.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.kZero));
    lockPositionTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));
    resetGyroTrigger.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    // shootTrigger.whileTrue(
    //     //   Commands.parallel(
    //     //     Commands.runEnd(() -> shooter.startShooter(), () -> shooter.stopShooter(), shooter),
    //     //     Commands.runEnd(() -> pitch.setAngle(30.0), () -> pitch.setAngle(0.0), pitch)
    //     //  )
    //     Commands.runEnd(() -> shooter.startShooter(), () -> shooter.stopShooter(), shooter));
   // controller.leftTrigger(0.25).whileTrue(() -> intake.runIntake(intake));
    intakeTrigger.whileTrue(
        Commands.runEnd(() -> intake.startIntake(), () -> intake.stopIntake(), intake));
    
    shootTrigger.whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> pitch.setAngle(10.0), pitch), 
            Commands.runOnce(() -> pitch.setAngle(10.0), pitch)
        )
        
    );

  }

  public Command AimThenShoot (){
    return Commands.sequence(
      Commands.runOnce(() -> pitch.setAngle(hoodAngle), pitch), 
      Commands.runOnce(() -> shooter.startShooter(), shooter) // TODO: Write a shooter function so we can set exitVelocity here
    );
    
  }

}
