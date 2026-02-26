// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPitchConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterPitch extends SubsystemBase {
  /* Hardware */
  private final TalonFX pitchMotor = new TalonFX(ShooterPitchConstants.MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride = new LoggedNetworkBoolean("Pitch Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Pitch Manual Duty", 0.0);
  private double targetPositionRotations = 0;

  public ShooterPitch() {
    configureHardware();
    pitchMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(ShooterPitchConstants.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(ShooterPitchConstants.STATOR_AMP_LIMIT));
    
    config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(ShooterPitchConstants.PEAK_FORWARD_TORQUE_CURRENT))
                        .withPeakReverseTorqueCurrent(Amps.of(ShooterPitchConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.MotionMagicCruiseVelocity = ShooterPitchConstants.MM_CRUISE_VEL;
    config.MotionMagic.MotionMagicAcceleration = ShooterPitchConstants.MM_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = ShooterPitchConstants.MM_JERK;

    config.Slot0.kP = ShooterPitchConstants.PID_KP;
    config.Slot0.kI = ShooterPitchConstants.PID_KI;
    config.Slot0.kD = ShooterPitchConstants.PID_KD;
    config.Slot0.kS = ShooterPitchConstants.PID_KS;
    config.Slot0.kV = ShooterPitchConstants.PID_KV;

    applyConfig(pitchMotor, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  // Active Control for Prod
  public void setTargetPosition(double rotations) {
    if (LNNOverride.getAsBoolean()) return;

    this.targetPositionRotations = rotations;
    pitchMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // Manual Control for Dev
  public void setManualDutyCycle(double output) {
    this.targetPositionRotations = pitchMotor.getPosition().getValueAsDouble(); 
    pitchMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    pitchMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    Logger.recordOutput("ShooterPitch/TargetRotations", targetPositionRotations);
    Logger.recordOutput("ShooterPitch/ActualRotations", pitchMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("ShooterPitch/StatorCurrent", pitchMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean readyToShoot() {
    double currentPos = pitchMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations) <= ShooterPitchConstants.PITCH_MOE;
  }
}