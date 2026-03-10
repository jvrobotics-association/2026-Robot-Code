// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.LeftMotor.CAN_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.RightMotor.CAN_ID, "rio");

  /* Control Requests - Distinct objects for separate motor streams */
  private final MotionMagicVelocityTorqueCurrentFOC leftVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicVelocityTorqueCurrentFOC rightVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  /* State */
  private final LoggedNetworkBoolean LNNOverride =
      new LoggedNetworkBoolean("Shooter Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Shooter Speed", 0.0);

  private double targetVelocityRPS = 0;

  /** Creates a new Shooter */
  public Shooter() {
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
        .withStatorCurrentLimit(Amps.of(ShooterConstants.LeftMotor.STATOR_AMP_LIMIT));

    config.Slot0.withKP(ShooterConstants.LeftMotor.PID_KP)
        .withKV(ShooterConstants.LeftMotor.PID_KV)
        .withKS(ShooterConstants.LeftMotor.PID_KS);

    config.MotionMagic.withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(ShooterConstants.LeftMotor.MM_ACCELERATION))
        .withMotionMagicJerk(
            RotationsPerSecondPerSecond.per(Second).of(ShooterConstants.LeftMotor.MM_JERK));

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.RightMotor.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(ShooterConstants.RightMotor.STATOR_AMP_LIMIT));

    config.Slot0.withKP(ShooterConstants.RightMotor.PID_KP)
        .withKV(ShooterConstants.RightMotor.PID_KV)
        .withKS(ShooterConstants.RightMotor.PID_KS);

    config.MotionMagic.withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(ShooterConstants.RightMotor.MM_ACCELERATION))
        .withMotionMagicJerk(
            RotationsPerSecondPerSecond.per(Second).of(ShooterConstants.RightMotor.MM_JERK));

    applyConfig(rightMotor, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  /** Sets target velocity */
  public void setTargetVelocity(double velocityRPS) {
    if (LNNOverride.getAsBoolean()) return;

    this.targetVelocityRPS = velocityRPS;
  }

  public void shoot() {
    // leftMotor.setControl(leftVelocityRequest.withVelocity(targetVelocityRPS));
    // rightMotor.setControl(rightVelocityRequest.withVelocity(targetVelocityRPS));

    leftMotor.setControl(leftVelocityRequest.withVelocity(ShooterConstants.SPEED));
    rightMotor.setControl(rightVelocityRequest.withVelocity(ShooterConstants.SPEED));

    // leftMotor.setControl(voltageRequest.withOutput(8));
    // rightMotor.setControl(voltageRequest.withOutput(8));
  }

  /** Sets open-loop duty cycle (for testing/override). */
  public void setDutyCycle(double output) {
    this.targetVelocityRPS = 0; // Invalidate velocity setpoint
    leftMotor.setControl(leftVelocityRequest.withVelocity(output));
    rightMotor.setControl(rightVelocityRequest.withVelocity(output));
  }

  public void stop() {
    this.targetVelocityRPS = 0;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Handle NetworkTable Override logic
    if (LNNOverride.getAsBoolean()) {
      setDutyCycle(LNNTarget.getAsDouble());
    }

    // AdvantageKit Logging (Crucial for the "Active" method to see what is happening)
    Logger.recordOutput("Shooter/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Shooter/LeftActualRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/RightActualRPS", rightMotor.getVelocity().getValueAsDouble());
  }

  public boolean readyToShoot() {
    if (targetVelocityRPS <= 0) return false;

    double currentLeft = leftMotor.getVelocity().getValueAsDouble();
    double currentRight = rightMotor.getVelocity().getValueAsDouble();

    // Use a flat minimum floor for tolerance to avoid noise issues at low speeds
    double tolerance = Math.max(targetVelocityRPS * ShooterConstants.SPEED_MOE, 0.5);

    return Math.abs(currentLeft - targetVelocityRPS) <= tolerance
        && Math.abs(currentRight - targetVelocityRPS) <= tolerance;
  }
}
