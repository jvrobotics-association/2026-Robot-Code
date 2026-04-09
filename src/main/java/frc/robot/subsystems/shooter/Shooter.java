// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, "rio");

  /* Control Requests - Distinct objects for separate motor streams */
  private final VelocityTorqueCurrentFOC leftVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public record ShooterCaluclatedValues(double velocity, double pitch) {}

  /** Creates a new Shooter */
  public Shooter() {
    configureLeftMotor();
    configureRightMotor();
  }

  private void configureLeftMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(ShooterConstants.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(ShooterConstants.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(ShooterConstants.PEAK_REVERSE_VOLTAGE);

    config.Slot0.withKP(ShooterConstants.PID_KP)
        .withKV(ShooterConstants.PID_KV)
        .withKS(ShooterConstants.PID_KS);

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLowerLimit(ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT)
        .withSupplyCurrentLowerTime(ShooterConstants.SUPPLY_CURRENT_LOWER_TIME);

    config.Voltage.withPeakForwardVoltage(ShooterConstants.PEAK_FORWARD_VOLTAGE)
        .withPeakReverseVoltage(ShooterConstants.PEAK_REVERSE_VOLTAGE);

    applyConfig(rightMotor, config);

    rightMotor.setControl(new StrictFollower(ShooterConstants.LEFT_MOTOR_ID));

    rightMotor.optimizeBusUtilization();
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void runShooter(double velocity) {
    leftMotor.setControl(leftVelocityRequest.withVelocity(velocity));
  }

  public void staticShoot() {
    leftMotor.setControl(leftVelocityRequest.withVelocity(ShooterConstants.CLOSE_SHOT.get(85.2)));
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Shooter/TargetVelocityRPS", leftMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput("Shooter/LeftActualRPS", leftMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput(
    //     "Shooter/LeftStatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Shooter/LeftSupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Shooter/RightActualRPS", rightMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput(
    //     "Shooter/RightStatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Shooter/RightSupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
  }

  public boolean atTargetVelocity() {
    // return leftMotor.getVelocity().getValueAsDouble() >=
    // leftMotor.getVelocity().getValueAsDouble() + ShooterConstants.RPS_TOLERANCE;
    return Math.abs(leftMotor.getClosedLoopError().getValueAsDouble())
        <= ShooterConstants.RPS_TOLERANCE;
  }
}
