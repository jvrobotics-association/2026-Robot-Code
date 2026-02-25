// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /************ Declare Motors ************/
  private static final TalonFX leftMotor = new TalonFX(ShooterConstants.LeftMotor.CAN_ID, "rio");

  private static final TalonFX rightMotor = new TalonFX(ShooterConstants.RightMotor.CAN_ID, "rio");

  /************ Declare Configs ************/
  final TalonFXConfiguration leftMotorConfig;

  final TalonFXConfiguration rightMotorConfig;
  final MotionMagicConfigs leftMotorMMConfigs;
  final MotionMagicConfigs rightMotorMMConfigs;
  final Slot0Configs rightMotorSlot0;
  final Slot0Configs leftMotorSlot0;

  /************ Motor Control Requests ************/

  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  /************ Class Member Variables ************/
  private final LoggedNetworkNumber ShooterSpeed = new LoggedNetworkNumber("Shooter Speed", 0.0);

  private double flywheelSetpoint = 0;

  /** Creates a new Shooter */
  public Shooter() {

    /************ Configure Left Motor ************/

    leftMotorConfig = new TalonFXConfiguration();
    // Gets feed from either the encoder or the CANcoder
    leftMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .SensorToMechanismRatio = ShooterConstants.LeftMotor.SENSOR_TO_MECH; // TODO: Verify if this or IntakeMotor's config works better
    // set Neutral Mode to Coast
    leftMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    // Limits the amount of Amps the motor can draw
    // Volts * Amps = Watts
    leftMotorConfig
      .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(ShooterConstants.LeftMotor.STATOR_LIMIT));
    leftMotorConfig
      .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(ShooterConstants.LeftMotor.PEAK_FORWARD_TORQUE));

    leftMotorSlot0 = leftMotorConfig.Slot0;
    leftMotorSlot0.kS = ShooterConstants.LeftMotor.PID_KS; // Add 0.25 V output to overcome static friction
    leftMotorSlot0.kV = ShooterConstants.LeftMotor.PID_KV; // A velocity target of 1 rps results in 0.12 V output
    leftMotorSlot0.kA = ShooterConstants.LeftMotor.PID_KA; // An acceleration of 1 rps/s requires 0.01 V output
    leftMotorSlot0.kP = ShooterConstants.LeftMotor.PID_KP; // A position error of 0.2 rotations results in 12 V output
    leftMotorSlot0.kI = ShooterConstants.LeftMotor.PID_KI; // No output for integrated error
    leftMotorSlot0.kD = ShooterConstants.LeftMotor.PID_KD; // A velocity error of 1 rps results in 0.5 V output

    // Configures the Acceleration,Torque, and Stator limits
    leftMotorMMConfigs = leftMotorConfig.MotionMagic;
    leftMotorMMConfigs
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ShooterConstants.LeftMotor.MM_ACCELERATION))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ShooterConstants.LeftMotor.MM_JERK));
    // Apply the left shooter motor config, retry config apply up to 5 times, report if failure
    StatusCode leftMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      leftMotorStatus = leftMotor.getConfigurator().apply(leftMotorConfig);
      if (leftMotorStatus.isOK()) break;
    }
    if (!leftMotorStatus.isOK()) {
      System.out.println(
          "Could not apply left shooter motor config, error code: " + leftMotorStatus.toString());
    }

    /************ Configure Right Motor ************/

    rightMotorConfig = new TalonFXConfiguration();
    // Gets feed from either the encoder or the CANcoder
    rightMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .SensorToMechanismRatio =
        1;
    // set Neutral Mode to Coast
    rightMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    // Limits the amount of Amps the motor can draw
    // Volts * Amps = Watts
    rightMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));

    // Configures the Cruise, Acceleration,Torque, and Stator limits of the right motor
    rightMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    rightMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    rightMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));

    rightMotorSlot0 = rightMotorConfig.Slot0;
    rightMotorSlot0.kS = 0.0; // Add 0.25 V output to overcome static friction
    rightMotorSlot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    rightMotorSlot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    rightMotorSlot0.kP = 0.0; // A position error of 0.2 rotations results in 12 V output
    rightMotorSlot0.kI = 0.0; // No output for integrated error
    rightMotorSlot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output

    rightMotorMMConfigs = rightMotorConfig.MotionMagic;
    rightMotorMMConfigs
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    // Apply the right shooter motor config, retry config apply up to 5 times, report if failure
    StatusCode rightMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      rightMotorStatus = rightMotor.getConfigurator().apply(rightMotorConfig);
      if (rightMotorStatus.isOK()) break;
    }
    if (!rightMotorStatus.isOK()) {
      System.out.println(
          "Could not apply right shooter motor config, error code: " + rightMotorStatus.toString());
    }
  }

  @Override
  public void periodic() {
    leftMotor.setControl(m_manualRequest.withOutput(flywheelSetpoint));
    rightMotor.setControl(m_manualRequest.withOutput(flywheelSetpoint));
  }
  // Sets the speed and makes it a double
  public void startShooter() {
    leftMotor.setControl(m_manualRequest.withOutput(ShooterSpeed.getAsDouble()));
    rightMotor.setControl(m_manualRequest.withOutput(ShooterSpeed.getAsDouble()));
  }
  // Sets the speed and makes it a double
  public void stopShooter() {
    leftMotor.setControl(m_manualRequest.withOutput(0.0));
    rightMotor.setControl(m_manualRequest.withOutput(0.0));
  }

  public void setSpeed(double speed) {
    this.flywheelSetpoint = speed; // TODO: Limit updates to a margin of error (i.e. 2%)
  }

  public boolean readyToShoot() {
    boolean leftReady = false;
    boolean rightReady = false;

    leftReady =
        (leftMotor.getPosition().getValueAsDouble()
                >= ((1.00 - ShooterConstants.SPEED_MOE)) * flywheelSetpoint)
            && (leftMotor.getPosition().getValueAsDouble()
                <= ((1.00 + ShooterConstants.SPEED_MOE) * flywheelSetpoint));

    rightReady =
        (rightMotor.getPosition().getValueAsDouble()
                >= ((1.00 - ShooterConstants.SPEED_MOE)) * flywheelSetpoint)
            && (rightMotor.getPosition().getValueAsDouble()
                <= ((1.00 + ShooterConstants.SPEED_MOE) * flywheelSetpoint));

    return leftReady && rightReady;
  }
}
