// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;
import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  /** LEFT MOTOR */
  private static final TalonFX leftClimberMotor = new TalonFX(climberConstants.LEFT_MOTOR, "rio");
  private static final TalonFX rightClimberMotor = new TalonFX(climberConstants.LEFT_MOTOR, "rio");
  private static final CANcoder leftEncoder = new CANcoder(climberConstants.LEFT_ENCODER, "rio");
  private static final CANcoder rightEncoder = new CANcoder(climberConstants.LEFT_ENCODER, "rio");

  final TalonFXConfiguration leftClimberMotorConfig;
  final DutyCycleOut m_leftmanualRequest = new DutyCycleOut(0);
  final TalonFXConfiguration rightClimberMotorConfig;
  final DutyCycleOut m_rightmanualRequest = new DutyCycleOut(0);
  public final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
  /** Creates a new Climber. */
  public Climber() {

    /** LEFT MOTOR */
    leftClimberMotorConfig = new TalonFXConfiguration();

    leftClimberMotorConfig
        .Feedback
        .withFeedbackRemoteSensorID(climberConstants.LEFT_ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1) // TODO: Set Sensor to Mechanism Ratio
        .withRotorToSensorRatio(1); // TODO: Set Rotor to Sensor Ratio
    
    // Sets the Neutral Mode to Brake
    leftClimberMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    leftClimberMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));
    //Configures the Cruise, Accelerartion, Torque, and Stator limts
    leftClimberMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        0.0; // TODO: Set Cruise Velocity
    leftClimberMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; // TODO: Set Acceleration
    leftClimberMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    leftClimberMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Apply the intake extension config, retry config apply up to 5 times, report if failure
    StatusCode leftMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      leftMotorStatus = leftClimberMotor.getConfigurator().apply(leftClimberMotorConfig);
      if (leftMotorStatus.isOK()) break;
    }

    if (!leftMotorStatus.isOK()) {
      System.out.println(
          "Could not apply Left Climber motor config, error code: " + leftMotorStatus.toString());
    }

    CANcoderConfiguration leftEncoderConfig = new CANcoderConfiguration();

    leftEncoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    // Apply the encoder config, retry config apply up to 5 times, report if failure
    StatusCode leftEncoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      leftEncoderStatus = leftEncoder.getConfigurator().apply(leftEncoderConfig);
      if (leftEncoderStatus.isOK()) break;
    }
    if (!leftEncoderStatus.isOK()) {
      System.out.println("Could not apply left encoder config, error code: " + leftEncoderStatus.toString());
    }
    
    /** RIGHT MOTOR */
    rightClimberMotorConfig = new TalonFXConfiguration();

    rightClimberMotorConfig
        .Feedback
        .withFeedbackRemoteSensorID(climberConstants.RIGHT_ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1) // TODO: Set Sensor to Mechanism Ratio
        .withRotorToSensorRatio(1); // TODO: Set Rotor to Sensor Ratio
    
    // Sets the Neutral Mode to Brake
    rightClimberMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    rightClimberMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));
    //Configures the Cruise, Accelerartion, Torque, and Stator limts
    rightClimberMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        0.0; // TODO: Set Cruise Velocity
    rightClimberMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; // TODO: Set Acceleration
    rightClimberMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    rightClimberMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Apply the intake extension config, retry config apply up to 5 times, report if failure
    StatusCode rightMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      rightMotorStatus = rightClimberMotor.getConfigurator().apply(rightClimberMotorConfig);
      if (rightMotorStatus.isOK()) break;
    }

    if (!rightMotorStatus.isOK()) {
      System.out.println(
          "Could not apply Right Climber motor config, error code: " + rightMotorStatus.toString());
    }

    CANcoderConfiguration rightEncoderConfig = new CANcoderConfiguration();

    rightEncoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    // Apply the encoder config, retry config apply up to 5 times, report if failure
    StatusCode rightEncoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      rightEncoderStatus = rightEncoder.getConfigurator().apply(rightEncoderConfig);
      if (rightEncoderStatus.isOK()) break;
    }
    if (!rightEncoderStatus.isOK()) {
      System.out.println("Could not apply right encoder config, error code: " + rightEncoderStatus.toString());
    }

    // Reset the position that the elevator currently is at to 0.
    // The physical elevator should be all the way down when this is set.
    rightClimberMotor.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
