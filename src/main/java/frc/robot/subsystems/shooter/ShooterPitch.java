// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPitchConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterPitch extends SubsystemBase {
  /* Hardware */
  private final TalonFXS pitchMotor = new TalonFXS(ShooterPitchConstants.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  public ShooterPitch() {
    configureHardware();
    pitchMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    config.Commutation.withMotorArrangement(MotorArrangementValue.Minion_JST);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(ShooterPitchConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ShooterPitchConstants.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(ShooterPitchConstants.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(ShooterPitchConstants.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(ShooterPitchConstants.PEAK_REVERSE_VOLTAGE);

    config.ExternalFeedback.withSensorToMechanismRatio(74);

    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ShooterPitchConstants.MAX_ROTATION)
        .withReverseSoftLimitThreshold(ShooterPitchConstants.MIN_ROTATION);

    config.Slot0.withKP(ShooterPitchConstants.PID_KP)
        .withKD(ShooterPitchConstants.PID_KD)
        .withKS(ShooterPitchConstants.PID_KS)
        .withKV(ShooterPitchConstants.PID_KV);

    config.MotionMagic.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(ShooterPitchConstants.MM_CRUISE_VEL))
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(ShooterPitchConstants.MM_ACCELERATION));

    applyConfig(config);
  }

  private void applyConfig(TalonFXSConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pitchMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void aim(double position) {
    pitchMotor.setControl(positionRequest.withPosition(position));
  }

  public void stop() {
    pitchMotor.setControl(positionRequest.withPosition(0));
  }

  public boolean isAtPosition() {
    return Math.abs(pitchMotor.getClosedLoopError().getValueAsDouble())
        <= ShooterPitchConstants.TOLERANCE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "ShooterPitch/TargetRotations", pitchMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(
        "ShooterPitch/ActualRotations", pitchMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "ShooterPitch/StatorCurrent", pitchMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("ShooterPitch/isAtPosition", isAtPosition());
  }
}
