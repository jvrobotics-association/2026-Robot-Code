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
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private double targetVelocityRPS = 0;

  public Intake() {
    configureHardware();
    intakeMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(Seconds.of(IntakeConstants.SUPPLY_CURRENT_LOWER_TIME));

    config.Voltage.withPeakForwardVoltage(IntakeConstants.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeConstants.PEAK_REVERSE_VOLTAGE);

    config.Slot0.withKS(IntakeConstants.PID_KS)
        .withKV(IntakeConstants.PID_KV)
        .withKP(IntakeConstants.PID_KP);

    config.MotionMagic.withMotionMagicAcceleration(
        RotationsPerSecondPerSecond.of(IntakeConstants.MM_ACCELERATION));

    applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply intake motor config, error code: " + status.toString());
    }
  }

  // Prod control
  public void startIntake() {
    this.targetVelocityRPS = IntakeConstants.INTAKE_SPEED;
    intakeMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Dev control
  public void setManualDutyCycle(double output) {
    intakeMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stopIntake() {
    this.targetVelocityRPS = 0;
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // AdvantageKit Logging
    Logger.recordOutput("Intake/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Intake/ActualVelocityRPS", intakeMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/StatorCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
  }
}
