// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPitchConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterPitch extends SubsystemBase {
  /* Hardware */
  private final TalonFXS pitchMotor = new TalonFXS(ShooterPitchConstants.CAN_ID, "rio");

  /* Control Requests */
  private final MotionMagicDutyCycle positionRequest = new MotionMagicDutyCycle(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  /* State */
  private final LoggedNetworkBoolean LNNOverride =
      new LoggedNetworkBoolean("Pitch Override", false);
  private final LoggedNetworkBoolean LNNConfig =
      new LoggedNetworkBoolean("Pitch Config Applied", false);
  private final LoggedNetworkNumber LNNTarget = new LoggedNetworkNumber("Pitch Setpoint", 0.0);
  private final LoggedNetworkNumber LNNCurrent = new LoggedNetworkNumber("Pitch Current", 0.0);
  private double targetPositionRotations = 0;

  public ShooterPitch() {
    configureHardware();
    pitchMotor.setPosition(0);
  }

  private void configureHardware() {
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    config.Commutation.withMotorArrangement(MotorArrangementValue.Minion_JST);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits.withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(ShooterPitchConstants.STATOR_AMP_LIMIT));

    // TODO: Voltage Condigs? External Feedback configs?
    config.Slot0.kP = ShooterPitchConstants.PID_KP;
    config.Slot0.kI = ShooterPitchConstants.PID_KI;
    config.Slot0.kD = ShooterPitchConstants.PID_KD;
    config.Slot0.kS = ShooterPitchConstants.PID_KS;
    config.Slot0.kV = ShooterPitchConstants.PID_KV;

    MotionMagicConfigs MMConf = config.MotionMagic;
    MMConf.withMotionMagicCruiseVelocity(RotationsPerSecond.of(ShooterPitchConstants.MM_CRUISE_VEL));
    MMConf.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ShooterPitchConstants.MM_ACCELERATION));
    // config.MotionMagic.MotionMagicJerk = ShooterPitchConstants.MM_JERK;

    applyConfig(pitchMotor, config);
  }

  private void applyConfig(TalonFXS motor, TalonFXSConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        LNNConfig.set(true);
        break;
      }
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
    this.targetPositionRotations = 0;
    LNNCurrent.set(pitchMotor.getPosition().getValueAsDouble());
    pitchMotor.setControl(positionRequest.withPosition(output));
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
    Logger.recordOutput(
        "ShooterPitch/ActualRotations", pitchMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "ShooterPitch/StatorCurrent", pitchMotor.getStatorCurrent().getValueAsDouble());
  }

  public boolean readyToShoot() {
    double currentPos = pitchMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPos - targetPositionRotations) <= ShooterPitchConstants.PITCH_MOE;
  }
}
