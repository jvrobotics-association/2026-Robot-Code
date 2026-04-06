// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(IntakeConstants.RightMotor.MOTOR_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(IntakeConstants.RightMotor.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private double targetVelocityRPS = 0;

  public Intake() {
    configureLeftMotor();
    configureRightMotor();
  }

  private void configureLeftMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.LeftMotor.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(
            Seconds.of(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LOWER_TIME));

    config.Voltage.withPeakForwardVoltage(IntakeConstants.LeftMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeConstants.LeftMotor.PEAK_REVERSE_VOLTAGE);

    config.Slot0.withKS(IntakeConstants.LeftMotor.PID_KS)
        .withKV(IntakeConstants.LeftMotor.PID_KV)
        .withKP(IntakeConstants.LeftMotor.PID_KP);

    config.MotionMagic.withMotionMagicAcceleration(
        RotationsPerSecondPerSecond.of(IntakeConstants.LeftMotor.MM_ACCELERATION));

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.RightMotor.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeConstants.RightMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(IntakeConstants.RightMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(
            Seconds.of(IntakeConstants.RightMotor.SUPPLY_CURRENT_LOWER_TIME));

    config.Voltage.withPeakForwardVoltage(IntakeConstants.RightMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeConstants.RightMotor.PEAK_REVERSE_VOLTAGE);

    config.Slot0.withKS(IntakeConstants.RightMotor.PID_KS)
        .withKV(IntakeConstants.RightMotor.PID_KV)
        .withKP(IntakeConstants.RightMotor.PID_KP);

    config.MotionMagic.withMotionMagicAcceleration(
        RotationsPerSecondPerSecond.of(IntakeConstants.RightMotor.MM_ACCELERATION));

    applyConfig(rightMotor, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  // Prod control
  public void startIntake() {
    this.targetVelocityRPS = IntakeConstants.INTAKE_SPEED;
    leftMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
    rightMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Dev control
  public void setManualDutyCycle(double output) {
    leftMotor.setControl(dutyCycleRequest.withOutput(output));
    rightMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stopIntake() {
    this.targetVelocityRPS = 0;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // AdvantageKit Logging
    Logger.recordOutput("Intake/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Intake/LeftVelocityRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Intake/LeftStatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/RightVelocityRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Intake/RightStatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
  }
}
