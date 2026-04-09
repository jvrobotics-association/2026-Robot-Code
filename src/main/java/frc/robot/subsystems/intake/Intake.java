// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(IntakeConstants.LeftMotor.MOTOR_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(IntakeConstants.RightMotor.MOTOR_ID, "rio");

  /* Control Requests */
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public Intake() {
    configureLeftMotor();
    configureRightMotor();
  }

  private void configureLeftMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.LeftMotor.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(Seconds.of(0.25));

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(
            Seconds.of(IntakeConstants.LeftMotor.SUPPLY_CURRENT_LOWER_TIME));

    config.Voltage.withPeakForwardVoltage(IntakeConstants.LeftMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeConstants.LeftMotor.PEAK_REVERSE_VOLTAGE);

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.RightMotor.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(Seconds.of(0.25));

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeConstants.RightMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(IntakeConstants.RightMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(
            Seconds.of(IntakeConstants.RightMotor.SUPPLY_CURRENT_LOWER_TIME));

    config.Voltage.withPeakForwardVoltage(IntakeConstants.RightMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeConstants.RightMotor.PEAK_REVERSE_VOLTAGE);

    applyConfig(rightMotor, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void runIntake(double output) {
    leftMotor.setControl(dutyCycleRequest.withOutput(output));
    rightMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stopIntake() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // AdvantageKit Logging
    Logger.recordOutput("Intake/LeftActualRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/RightActualRPS", leftMotor.getVelocity().getValueAsDouble());
  }
}
