// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Tower extends SubsystemBase {
  /* Hardware */
  private final TalonFX towerMotor = new TalonFX(TowerConstants.MOTOR, "rio");

  /* Control Requests */
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride = new LoggedNetworkBoolean("Tower Override", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Tower Manual Duty", 0.0);
  
  private double targetVelocityRPS = 0;

  /** Creates a new Tower. */
  public Tower() {
    configureHardware();
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(TowerConstants.SENSOR_TO_MECH_RATIO);
    
    config.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast) 
        .withInverted(InvertedValue.Clockwise_Positive); // TODO: VERIFY
    
    config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(TowerConstants.FORWARD_TORQUE_AMPS_LIMIT));
    config.CurrentLimits.withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_AMP_LIMIT))
                        .withStatorCurrentLimitEnable(true);

    config.Slot0.kP = TowerConstants.PID_KP;
    config.Slot0.kS = TowerConstants.PID_KS;
    config.Slot0.kV = TowerConstants.PID_KV;

    // Motion Magic Profile
    config.MotionMagic
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(TowerConstants.MM_ACCELERATION));
        applyConfig(config);
  }

  private void applyConfig(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = towerMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Tower motor config, error: " + status.toString());
    }
  }

  // Prod - preset speed
  public void feed() {
    if (LNNOverride.getAsBoolean()) return;

    this.targetVelocityRPS = TowerConstants.TOWER_SPEED;
    towerMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Prod - Controllable speed
  public void setVelocity(double velocityRPS) {
    if (LNNOverride.getAsBoolean()) return;

    this.targetVelocityRPS = velocityRPS;
    towerMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
  }

  // Dev 
  public void setManualDutyCycle(double output) {
    this.targetVelocityRPS = 0;
    towerMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    this.targetVelocityRPS = 0;
    towerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if (LNNOverride.getAsBoolean()) {
      setManualDutyCycle(LNNTarget.getAsDouble());
    }

    Logger.recordOutput("Tower/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Tower/ActualVelocityRPS", towerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Tower/StatorCurrent", towerMotor.getStatorCurrent().getValueAsDouble());
  }
}