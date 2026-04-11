// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeExtensionConstants;
import frc.robot.Constants.IntakeExtensionConstants.ExtensionEncoder;
import frc.robot.Constants.IntakeExtensionConstants.ExtensionMotor;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends SubsystemBase {
  /* Hardware */
  private final TalonFX extensionMotor = new TalonFX(ExtensionMotor.MOTOR_ID, "rio");
  private final CANcoder extensionEncoder = new CANcoder(ExtensionEncoder.SENSOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);

  public IntakeExtension() {
    configureEncoder();
    configureMotor();
    extensionMotor.setPosition(0);
  }

  private void configureEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(IntakeExtensionConstants.ExtensionEncoder.MAGNET_OFFSET)
        .withAbsoluteSensorDiscontinuityPoint(
            IntakeExtensionConstants.ExtensionEncoder.ABSOLUTE_SENSOR_DISCONTINUITY);

    applyEncoderConfig(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(IntakeExtensionConstants.ExtensionEncoder.SENSOR_ID)
        .withRotorToSensorRatio(IntakeExtensionConstants.ExtensionMotor.ROTOR_TO_SENSOR_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(IntakeExtensionConstants.ExtensionMotor.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(
            IntakeExtensionConstants.ExtensionMotor.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(
            IntakeExtensionConstants.ExtensionMotor.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(
            IntakeExtensionConstants.ExtensionMotor.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(IntakeExtensionConstants.ExtensionMotor.PEAK_REVERSE_VOLTAGE);

    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeExtensionConstants.ExtensionMotor.MAX_ROTATION)
        .withReverseSoftLimitThreshold(IntakeExtensionConstants.ExtensionMotor.MIN_ROTATION);

    config.Slot0.withKP(ExtensionMotor.PID_KP)
        .withKD(ExtensionMotor.PID_KD)
        .withKS(ExtensionMotor.PID_KS)
        .withKV(ExtensionMotor.PID_KV)
        .withKG(ExtensionMotor.PID_KG)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withGravityArmPositionOffset(ExtensionMotor.PID_ARM_OFFSET);

    config.MotionMagic.withMotionMagicCruiseVelocity(ExtensionMotor.MM_CRUISE_VEL)
        .withMotionMagicAcceleration(ExtensionMotor.MM_ACCELERATION);

    applyMotorConfig(config);
  }

  private void applyMotorConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = extensionMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply Intake Extension motor config, error: " + status.toString());
    }
  }

  private void applyEncoderConfig(CANcoderConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = extensionEncoder.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply Intake Extension encoder config, error: " + status.toString());
    }
  }

  public void deploy() {
    extensionMotor.setControl(
        positionRequest.withPosition(IntakeExtensionConstants.DEPLOYED_ROTATIONS));
  }

  public void smartDeploy(BooleanSupplier hopperExtended) {
    if (hopperExtended.getAsBoolean()) deploy();
  }

  public void bumpRetract() {
    extensionMotor.setControl(
        positionRequest.withPosition(IntakeExtensionConstants.BUMP_ROTATIONS));
  }

  public void fullRetract() {
    extensionMotor.setControl(
        positionRequest.withPosition(IntakeExtensionConstants.FULL_RETRACTED_ROTATIONS));
  }

  public void stop() {
    extensionMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "IntakeExtension/TargetRotations",
        extensionMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(
        "IntakeExtension/ActualRotations", extensionMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "IntakeExtension/StatorCurrent", extensionMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean isAtPosition() {
    return Math.abs(extensionMotor.getClosedLoopError().getValueAsDouble())
        <= IntakeExtensionConstants.TOLERANCE_ROTATIONS;
  }
}
