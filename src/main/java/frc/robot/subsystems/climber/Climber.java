// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Climber extends SubsystemBase {
  /* Hardware */
  private final TalonFX climberMotor = new TalonFX(ClimberConstants.MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride = new LoggedNetworkBoolean("Climber Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Climber Manual Duty", 0.0);
  
  private double targetPositionRotations = 0;

  public Climber() {
    configureHardware();
    climberMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                   .withSensorToMechanismRatio(ClimberConstants.SENSOR_TO_MECH_RATIO); 

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake); 
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(ClimberConstants.STATOR_AMP_LIMIT)); 
    
    config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(ClimberConstants.PEAK_FORWARD_TORQUE_CURRENT))
                        .withPeakReverseTorqueCurrent(Amps.of(ClimberConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MM_CRUISE_VEL;
    config.MotionMagic.MotionMagicAcceleration = ClimberConstants.MM_ACCELERATION;

    config.Slot0.kP = ClimberConstants.PID_KP;
    config.Slot0.kD = ClimberConstants.PID_KD;
    config.Slot0.kS = ClimberConstants.PID_KS;
    config.Slot0.kV = ClimberConstants.PID_KV;

    applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = climberMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Climber config, error: " + status.toString());
    }
  }

  // Moves climber up to setpoint
  public void prepareToClimb() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = ClimberConstants.PREPARE_ROTATIONS;
    climberMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // Moves climber down to setpoint
  public void climb() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = ClimberConstants.CLIMB_ROTATIONS; 
    climberMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // Active control
  public void setTargetPosition(double rotations) {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = rotations;
    climberMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // Dev control
  public void setManualDutyCycle(double output) {
    this.targetPositionRotations = climberMotor.getPosition().getValueAsDouble(); 
    climberMotor.setControl(positionRequest.withPosition(output));
  }

  public void stop() {
    climberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    Logger.recordOutput("Climber/TargetRotations", targetPositionRotations);
    Logger.recordOutput("Climber/ActualRotations", climberMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Climber/StatorCurrent", climberMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean isAtTarget() {
    double currentPos = climberMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations) <= ClimberConstants.TOLERANCE_ROTATIONS;
  }
}