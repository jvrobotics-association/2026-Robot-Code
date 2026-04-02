// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeExtensionConstants;
import frc.robot.Constants.IntakeExtensionConstants.ExtensionEncoder;
import frc.robot.Constants.IntakeExtensionConstants.ExtensionMotor;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends SubsystemBase {
  /* Hardware */
  private final TalonFX extensionMotor = new TalonFX(ExtensionMotor.MOTOR_ID, "rio");
  private final CANcoder extensionEncoder = new CANcoder(ExtensionEncoder.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);

  /* State */
  private double targetPositionRotations = 0;

  public IntakeExtension() {
    configureEncoder();
    configureMotor();
    extensionMotor.setPosition(0);
  }

  private void configureEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(ExtensionEncoder.MAGNET_OFFSET)
        .withAbsoluteSensorDiscontinuityPoint(ExtensionEncoder.ABSOLUTE_SENSOR_DISCONTINUITY);

    applyEncoderConfig(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(ExtensionEncoder.MOTOR_ID)
        .withRotorToSensorRatio(ExtensionMotor.ROTOR_TO_SENSOR_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    config.TorqueCurrent.withPeakForwardTorqueCurrent(
            Amps.of(ExtensionMotor.PEAK_FORWARD_TORQUE_CURRENT))
        .withPeakReverseTorqueCurrent(Amps.of(ExtensionMotor.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.withMotionMagicCruiseVelocity(ExtensionMotor.MM_CRUISE_VEL)
        .withMotionMagicAcceleration(ExtensionMotor.MM_ACCELERATION);

    config.Slot0.withKP(ExtensionMotor.PID_KP)
        .withKD(ExtensionMotor.PID_KD)
        .withKS(ExtensionMotor.PID_KS)
        .withKV(ExtensionMotor.PID_KV)
        .withKG(ExtensionMotor.PID_KG)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withGravityArmPositionOffset(ExtensionMotor.PID_ARM_OFFSET);

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
    this.targetPositionRotations = IntakeExtensionConstants.DEPLOYED_ROTATIONS;
    extensionMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  public void retract() {
    this.targetPositionRotations = IntakeExtensionConstants.RETRACTED_ROTATIONS; // Typically 0
    extensionMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  public void stop() {
    extensionMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakeExtension/TargetRotations", targetPositionRotations);
    Logger.recordOutput(
        "IntakeExtension/ActualRotations", extensionMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "IntakeExtension/StatorCurrent", extensionMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean isAtTarget() {
    double currentPos = extensionMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations)
        <= IntakeExtensionConstants.TOLERANCE_ROTATIONS;
  }
}
