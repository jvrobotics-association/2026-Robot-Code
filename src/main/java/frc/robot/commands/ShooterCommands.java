// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands{
  
  private ShooterCommands() {}
  
  public static Command runShooter(Shooter shooter, double speed){
    return Commands.runEnd(() -> shooter.setSpeed(speed), () -> shooter.setSpeed(0.0), shooter);
  }

}
