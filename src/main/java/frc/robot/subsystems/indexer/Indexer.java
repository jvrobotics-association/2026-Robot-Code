// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /* Hardware */
  private final TalonFXS indexerMotor = new TalonFXS(IndexerConstants.CAN_ID, "rio");

  /* Control Requests */
  private final DutyCycleOut velocityRequest =
      new DutyCycleOut(0);

  /* State */
  private double targetVelocityRPS = 0;

  public Indexer() {
    configureHardware();
  }

  private void configureHardware() {
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    config.Commutation.withMotorArrangement(MotorArrangementValue.Minion_JST);
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    // config.ExternalFeedback.withSensorToMechanismRatio(IndexerConstants.SENSOR_TO_MECH_RATIO);
    // config.CurrentLimits.withStatorCurrentLimit(Amps.of(IndexerConstants.STATOR_AMP_LIMIT))
    //     .withStatorCurrentLimitEnable(true);

    // config.Slot0.withKP(IndexerConstants.PID_KP)
    //     .withKS(IndexerConstants.PID_KS)
    //     .withKV(IndexerConstants.PID_KV);

    // config.MotionMagic.withMotionMagicAcceleration(
    //         RotationsPerSecondPerSecond.of(IndexerConstants.MM_ACCELERATION))
    //     .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(IndexerConstants.MM_JERK));

    applyConfig(config);
  }

  private void applyConfig(TalonFXSConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexerMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Indexer motor config, error: " + status.toString());
    }
  }

  public void feed() {
    indexerMotor.setControl(velocityRequest.withOutput(IndexerConstants.INDEXER_FRACTIONAL));
  }

  // // Manual Control for Dev
  // public void setManualDutyCycle(double output) {
  //   indexerMotor.setControl(velocityRequest.withVelocity(output));
  // }

  public void stop() {
    indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {

    // AdvantageKit Logging
    Logger.recordOutput("Indexer/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Indexer/ActualVelocityRPS", indexerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/StatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
  }
}
