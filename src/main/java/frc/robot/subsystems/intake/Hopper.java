// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hopper extends SubsystemBase {
  /* Hardware */
  private final TalonFX hopperMotor = new TalonFX(HopperConstants.MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride =
      new LoggedNetworkBoolean("Hopper Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Hopper Manual Duty", 0.0);

  private double targetPositionRotations = 0;

  public Hopper() {
    configureHardware();
    hopperMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(HopperConstants.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits.withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_AMP_LIMIT));

    config.TorqueCurrent.withPeakForwardTorqueCurrent(
            Amps.of(HopperConstants.PEAK_FORWARD_TORQUE_CURRENT))
        .withPeakReverseTorqueCurrent(Amps.of(HopperConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.MM_CRUISE_VEL;
    config.MotionMagic.MotionMagicAcceleration = HopperConstants.MM_ACCELERATION;

    config.Slot0.kP = HopperConstants.PID_KP;
    config.Slot0.kD = HopperConstants.PID_KD;
    config.Slot0.kS = HopperConstants.PID_KS;
    config.Slot0.kV = HopperConstants.PID_KV;

    applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hopperMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Hopper config, error: " + status.toString());
    }
  }

  // DEPLOY to EXTENDED position
  public void deploy() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = HopperConstants.DEPLOYED_ROTATIONS;
    hopperMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // RETRACT to STOWED position
  public void retract() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = HopperConstants.RETRACTED_ROTATIONS; // Typically 0
    hopperMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // PROD - Active Control
  public void setTargetPosition(double rotations) {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = rotations;
    hopperMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // DEV - manual control
  public void setManualDutyCycle(double output) {
    this.targetPositionRotations = hopperMotor.getPosition().getValueAsDouble();
    hopperMotor.setControl(positionRequest.withPosition(output));
  }

  public void stop() {
    hopperMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    Logger.recordOutput("Hopper/TargetRotations", targetPositionRotations);
    Logger.recordOutput("Hopper/ActualRotations", hopperMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Hopper/StatorCurrent", hopperMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean isAtTarget() {
    double currentPos = hopperMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations) <= HopperConstants.TOLERANCE_ROTATIONS;
  }
}
