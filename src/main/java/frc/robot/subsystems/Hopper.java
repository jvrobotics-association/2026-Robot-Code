// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  /* Hardware */
  private final TalonFX hopperMotor = new TalonFX(HopperConstants.MOTOR_ID, "rio");

  /* Control Requests */
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

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

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(HopperConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(HopperConstants.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(Seconds.of(HopperConstants.SUPPLY_CURRENT_LOWER_TIME));

    config.Slot0.withKP(HopperConstants.PID_KP)
        .withKD(HopperConstants.PID_KD)
        .withKS(HopperConstants.PID_KS)
        .withKV(HopperConstants.PID_KV);

    config.MotionMagic.withMotionMagicCruiseVelocity(HopperConstants.MM_CRUISE_VEL)
        .withMotionMagicAcceleration(HopperConstants.MM_ACCELERATION);

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
    hopperMotor.setControl(positionRequest.withPosition(HopperConstants.DEPLOYED_ROTATIONS));
  }

  // RETRACT to STOWED position
  public void retract() {
    hopperMotor.setControl(positionRequest.withPosition(HopperConstants.RETRACTED_ROTATIONS));
  }

  // Manual control
  public void setManualDutyCycle(double output) {
    hopperMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    hopperMotor.stopMotor();
  }

  public BooleanSupplier isExtended() {
    return () ->
        hopperMotor.getPosition().getValueAsDouble() >= HopperConstants.DEPLOYED_ROTATIONS - 0.2;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Hopper/TargetRotations", hopperMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput("Hopper/ActualRotations", hopperMotor.getPosition().getValueAsDouble());
    // Logger.recordOutput("Hopper/StatorCurrent",
    // hopperMotor.getStatorCurrent().getValueAsDouble());
  }
}
