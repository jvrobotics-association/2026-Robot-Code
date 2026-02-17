// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.util.ControlCalculations;
import frc.robot.util.ControlCalculations.LaunchCalc;

public class MidSystem extends SubsystemBase {
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;

  //Calls for the Alliance Color from Driver Station. If it can't get the Alliance it defaults to Blue
  Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? FieldConstants.HUB_BLUE : FieldConstants.HUB_RED;

  // Shooter Variables
  private boolean isShooting = false;
  private double hoodAngle = 0;
  private double exitVelocity = 0;
  private LaunchCalc launchData;
  private ChassisSpeeds robotSpeed;
  private Pose2d robotPose;

  /** Creates a new MidSystems. */
  public MidSystem(Shooter shooter, ShooterPitch pitch, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier) {
    this.shooter = shooter;
    this.pitch = pitch;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedSupplier = robotSpeedSupplier;
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
  }

  public Command AimThenShoot (){
    return Commands.sequence(
      Commands.runOnce(() -> pitch.setAngle(hoodAngle), pitch), 
      Commands.runOnce(() -> shooter.startShooter(), shooter) // TODO: Write a shooter function so we can set exitVelocity here
    );
    
  }

}
