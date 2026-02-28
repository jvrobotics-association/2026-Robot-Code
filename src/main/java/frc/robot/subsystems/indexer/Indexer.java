// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Indexer extends SubsystemBase {
  /* Hardware */
  private final TalonFX indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride =
      new LoggedNetworkBoolean("Indexer Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Indexer Manual Duty", 0.0);

  private double targetVelocityRPS = 0;

  public Indexer() {
    configureHardware();
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IndexerConstants.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(
            NeutralModeValue.Coast) // TODO: COAST TO PREVENT WEAR OR BRAKE TO STOP FASTER
        .withInverted(InvertedValue.Clockwise_Positive);

    config.Slot0.kP = IndexerConstants.PID_KP;
    config.Slot0.kI = IndexerConstants.PID_KI;
    config.Slot0.kD = IndexerConstants.PID_KD;
    config.Slot0.kS = IndexerConstants.PID_KS;
    config.Slot0.kV = IndexerConstants.PID_KV;

    config.MotionMagic.withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(IndexerConstants.MM_ACCELERATION))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(IndexerConstants.MM_JERK));

    config.TorqueCurrent.withPeakForwardTorqueCurrent(
        Amps.of(IndexerConstants.FORWARD_TORQUE_AMPS_LIMIT));
    config.CurrentLimits.withStatorCurrentLimit(Amps.of(IndexerConstants.STATOR_AMP_LIMIT))
        .withStatorCurrentLimitEnable(true);

    applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexerMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void feed() {
    if (LNNOverride.getAsBoolean()) return;

    this.targetVelocityRPS = IndexerConstants.INDEXER_SPEED;
    indexerMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Active Control for Prod
  public void setVelocity(double velocityRPS) {
    if (LNNOverride.getAsBoolean()) return;

    this.targetVelocityRPS = velocityRPS;
    indexerMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Manual Control for Dev
  public void setManualDutyCycle(double output) {
    this.targetVelocityRPS = 0;
    indexerMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    this.targetVelocityRPS = 0;
    indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Handling Dashboard Override
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    // AdvantageKit Logging
    Logger.recordOutput("Indexer/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Indexer/ActualVelocityRPS", indexerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/StatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
  }
}
