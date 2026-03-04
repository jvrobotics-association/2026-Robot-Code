// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeExtensionConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeExtension extends SubsystemBase {
  /* Hardware */
  private final TalonFX extensionMotor = new TalonFX(IntakeExtensionConstants.MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride = new LoggedNetworkBoolean("IntakeExt Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("IntakeExt Manual Duty", 0.0);
  
  private double targetPositionRotations = 0;

  public IntakeExtension() {
    configureHardware();
    extensionMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                   .withSensorToMechanismRatio(IntakeExtensionConstants.SENSOR_TO_MECH_RATIO); 

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake); 
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(IntakeExtensionConstants.STATOR_AMP_LIMIT)); 
    
    config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(IntakeExtensionConstants.PEAK_FORWARD_TORQUE_CURRENT))
                        .withPeakReverseTorqueCurrent(Amps.of(IntakeExtensionConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.MotionMagicCruiseVelocity = IntakeExtensionConstants.MM_CRUISE_VEL;
    config.MotionMagic.MotionMagicAcceleration = IntakeExtensionConstants.MM_ACCELERATION;
    //config.MotionMagic.MotionMagicJerk = IntakeExtensionConstants.MM_JERK;

    config.Slot0.kP = IntakeExtensionConstants.PID_KP;
    config.Slot0.kI = IntakeExtensionConstants.PID_KI;
    config.Slot0.kD = IntakeExtensionConstants.PID_KD;
    config.Slot0.kS = IntakeExtensionConstants.PID_KS;
    config.Slot0.kV = IntakeExtensionConstants.PID_KV;

    applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = extensionMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Intake Extension config, error: " + status.toString());
    }
  }

  public void deploy() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = IntakeExtensionConstants.DEPLOYED_ROTATIONS;
    extensionMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  public void retract() {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = IntakeExtensionConstants.RETRACTED_ROTATIONS; // Typically 0
    extensionMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // PROD
  public void setTargetPosition(double rotations) {
    if (LNNOverride.getAsBoolean()) return;
    this.targetPositionRotations = rotations;
    extensionMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  // DEV
  public void setManualDutyCycle(double output) {
    this.targetPositionRotations = extensionMotor.getPosition().getValueAsDouble(); 
    extensionMotor.setControl(positionRequest.withPosition(output));
  }

  public void stop() {
    extensionMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    Logger.recordOutput("IntakeExtension/TargetRotations", targetPositionRotations);
    Logger.recordOutput("IntakeExtension/ActualRotations", extensionMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("IntakeExtension/StatorCurrent", extensionMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean isAtTarget() {
    double currentPos = extensionMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations) <= IntakeExtensionConstants.TOLERANCE_ROTATIONS;
  }
}