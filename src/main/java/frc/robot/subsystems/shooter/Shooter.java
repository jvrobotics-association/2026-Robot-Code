// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.LeftMotor.MOTOR_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.RightMotor.MOTOR_ID, "rio");

  /* Control Requests - Distinct objects for separate motor streams */
  private final VelocityTorqueCurrentFOC leftVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  private double targetVelocityRPS = 0;

  private LoggedNetworkNumber LNN_RPS = new LoggedNetworkNumber("Shooter RPS", 0);

  private Supplier<Pose2d> poseSupplier;
  private Translation3d hubTarget;

  /** Creates a new Shooter */
  public Shooter(Supplier<Pose2d> poseSupplier, Translation3d hubTarget) {
    this.poseSupplier = poseSupplier;
    this.hubTarget = hubTarget;
    configureLeftMotor();
    configureRightMotor();
  }

  private void configureLeftMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.LeftMotor.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(ShooterConstants.LeftMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ShooterConstants.LeftMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(ShooterConstants.LeftMotor.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(ShooterConstants.LeftMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(ShooterConstants.LeftMotor.PEAK_REVERSE_VOLTAGE);

    config.Slot0.withKP(ShooterConstants.LeftMotor.PID_KP)
        .withKV(ShooterConstants.LeftMotor.PID_KV)
        .withKS(ShooterConstants.LeftMotor.PID_KS);

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.RightMotor.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(ShooterConstants.RightMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ShooterConstants.RightMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(ShooterConstants.RightMotor.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(ShooterConstants.RightMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(ShooterConstants.RightMotor.PEAK_REVERSE_VOLTAGE);

    applyConfig(rightMotor, config);

    rightMotor.setControl(new StrictFollower(ShooterConstants.LeftMotor.MOTOR_ID));

    rightMotor.optimizeBusUtilization();
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void shoot() {
    targetVelocityRPS = LNN_RPS.getAsDouble();
    leftMotor.setControl(leftVelocityRequest.withVelocity(LNN_RPS.getAsDouble()));
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // AdvantageKit Logging (Crucial for the "Active" method to see what is happening)
    Logger.recordOutput("Shooter/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Shooter/LeftActualRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/LeftStatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/LeftSupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/RightActualRPS", rightMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/RightStatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/RightSupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/HubDistance",
        Inches.convertFrom(
            poseSupplier.get().getTranslation().getDistance(hubTarget.toTranslation2d()), Meters));
  }

  // public boolean readyToShoot() {
  //   if (targetVelocityRPS <= 0) return false;

  //   double currentLeft = leftMotor.getVelocity().getValueAsDouble();
  //   double currentRight = rightMotor.getVelocity().getValueAsDouble();

  //   // Use a flat minimum floor for tolerance to avoid noise issues at low speeds
  //   double tolerance = Math.max(targetVelocityRPS * ShooterConstants.SPEED_MOE, 0.5);

  //   return Math.abs(currentLeft - targetVelocityRPS) <= tolerance
  //       && Math.abs(currentRight - targetVelocityRPS) <= tolerance;
  // }
}
