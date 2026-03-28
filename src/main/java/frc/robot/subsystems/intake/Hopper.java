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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* Hardware */
  private final TalonFX hopperMotor = new TalonFX(HopperConstants.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private double targetPositionRotations = 0;

  public Hopper() {
    configureHardware();
    hopperMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(HopperConstants.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_AMP_LIMIT));

    config.TorqueCurrent.withPeakForwardTorqueCurrent(
            Amps.of(HopperConstants.PEAK_FORWARD_TORQUE_CURRENT))
        .withPeakReverseTorqueCurrent(Amps.of(HopperConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.withMotionMagicCruiseVelocity(HopperConstants.MM_CRUISE_VEL)
        .withMotionMagicAcceleration(HopperConstants.MM_ACCELERATION);

    config.Slot0.withKP(HopperConstants.PID_KP)
        .withKD(HopperConstants.PID_KD)
        .withKS(HopperConstants.PID_KS)
        .withKV(HopperConstants.PID_KV);

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
    this.targetPositionRotations = HopperConstants.DEPLOYED_ROTATIONS;
    hopperMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // RETRACT to STOWED position
  public void retract() {
    this.targetPositionRotations = HopperConstants.RETRACTED_ROTATIONS; // Typically 0
    hopperMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // DEV - manual control
  public void setManualDutyCycle(double output) {
    this.targetPositionRotations = hopperMotor.getPosition().getValueAsDouble();
    hopperMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    hopperMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hopper/TargetRotations", targetPositionRotations);
    Logger.recordOutput("Hopper/ActualRotations", hopperMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Hopper/StatorCurrent", hopperMotor.getStatorCurrent().getValueAsDouble());
  }
}
