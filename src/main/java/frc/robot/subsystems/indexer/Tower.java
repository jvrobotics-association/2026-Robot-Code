// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;
import org.littletonrobotics.junction.Logger;

public class Tower extends SubsystemBase {
  /* Hardware */
  private final TalonFX towerMotor = new TalonFX(TowerConstants.CAN_ID, "rio");

  /* Control Requests */
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private double targetVelocityRPS = 0;

  /** Creates a new Tower. */
  public Tower() {
    configureHardware();
  }

  private void configureHardware() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(TowerConstants.SENSOR_TO_MECH_RATIO);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_AMP_LIMIT));

    config.TorqueCurrent.withPeakForwardTorqueCurrent(
            Amps.of(TowerConstants.PEAK_FORWARD_TORQUE_CURRENT))
        .withPeakReverseTorqueCurrent(Amps.of(TowerConstants.PEAK_REVERSE_TORQUE_CURRENT));

    config.MotionMagic.withMotionMagicAcceleration(TowerConstants.MM_ACCELERATION);

    config.Slot0.withKP(TowerConstants.PID_KP)
        .withKS(TowerConstants.PID_KS)
        .withKV(TowerConstants.PID_KV);

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
    Logger.recordOutput("Tower/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Tower/ActualVelocityRPS", towerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Tower/StatorCurrent", towerMotor.getStatorCurrent().getValueAsDouble());
  }
}
