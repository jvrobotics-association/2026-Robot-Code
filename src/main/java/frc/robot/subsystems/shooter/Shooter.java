// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /************ Declare Motors ************/
  private static final TalonFX leftMotor = new TalonFX(shooterConstants.LEFT_MOTOR, "rio");

  private static final TalonFX rightMotor = new TalonFX(shooterConstants.RIGHT_MOTOR, "rio");

  /************ Declare Configs ************/
  final TalonFXConfiguration leftMotorConfig;

  final TalonFXConfiguration rightMotorConfig;

  /************ Motor Control Requests ************/
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

  /************ Class Member Variables ************/
  private final LoggedNetworkNumber ShooterSpeed = new LoggedNetworkNumber("Shooter Speed", 0.0);

  /** Creates a new Shooter */
  public Shooter() {

    /************ Configure Left Motor ************/

    leftMotorConfig = new TalonFXConfiguration();
    // Gets feed from either the encoder or the CANcoder
    leftMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
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
        .withStatorCurrentLimit(Amps.of(20));

    // Configures the Cruise, Acceleration,Torque, and Stator limits
    leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0; // TODO: set this value
    leftMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; // TODO: set this value
    leftMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    leftMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

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
    rightMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
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
    rightMotorConfig.MotionMagic.withMotionMagicCruiseVelocity(
        RotationsPerSecond.of(0.0)); // TODO: set this value
    rightMotorConfig.MotionMagic.withMotionMagicAcceleration(
        RotationsPerSecondPerSecond.of(0)); // TODO: set this value
    rightMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    rightMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

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
    // This method will be called once per scheduler run
  }
  // Sets the speed and makes it a double
  public void startShooter() {
    leftMotor.setControl(m_request.withVelocity(ShooterSpeed.getAsDouble()));
    rightMotor.setControl(m_request.withVelocity(ShooterSpeed.getAsDouble()));
  }
  // Sets the speed and makes it a double
  public void stopShooter() {
    leftMotor.setControl(m_request.withVelocity(0.0));
    rightMotor.setControl(m_request.withVelocity(0.0));
  }
}
