// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import edu.wpi.first.units.measure.Angle;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command runShooter(Shooter shooter) {
    return Commands.runEnd(() -> shooter.startShooter(), () -> shooter.stopShooter(), shooter);
  }

  public static Command AimAndScore(Shooter shooter,ShooterPitch shooterPitch, Angle position) {
    return Commands.parallel(
      Commands.runEnd(() -> shooter.startShooter(), () -> shooter.stopShooter(), shooter),
      Commands.runEnd(() -> shooterPitch.setAngle(position), () -> shooter.stopShooter(), shooterPitch)
    );
  }
}
