// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import frc.robot.Constants.intakeExtensionConstants;
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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtension extends SubsystemBase {
  private static final TalonFX intakeExtensionMotor = new TalonFX(intakeExtensionConstants.MOTOR, "rio");
  private static final CANcoder encoder = new CANcoder(intakeExtensionConstants.ENCODER, "rio");
  
  final TalonFXConfiguration intakeExtensionMotorConfig;
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  final public MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  /** Creates a new IntakeExtension. */
  public IntakeExtension() {
    intakeExtensionMotorConfig = new TalonFXConfiguration();

    intakeExtensionMotorConfig
      .Feedback
      .withFeedbackRemoteSensorID(intakeExtensionConstants.ENCODER)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
      .withSensorToMechanismRatio(1) // TODO: Set Sensor to Mechanism Ratio
      .withRotorToSensorRatio(1); // TODO: Set Rotor to Sensor Ratio
    intakeExtensionMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    intakeExtensionMotorConfig
      .CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(Amps.of(20));

    intakeExtensionMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0; // TODO: Set Cruise Velocity
    intakeExtensionMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; // TODO: Set Acceleration
    intakeExtensionMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    intakeExtensionMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Apply the intake extension config, retry config apply up to 5 times, report if failure
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
      if (motorStatus.isOK()) break;
    }

    if (!motorStatus.isOK()) {
      System.out.println("Could not apply Intake Extension motor config, error code: " + motorStatus.toString());
    }
    
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    // Apply the encoder config, retry config apply up to 5 times, report if failure
    StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      encoderStatus = encoder.getConfigurator().apply(encoderConfig);
      if (encoderStatus.isOK()) break;
    }
    if (!encoderStatus.isOK()) {
      System.out.println("Could not apply encoder config, error code: " + encoderStatus.toString());
    }

    // Reset the position that the elevator currently is at to 0.
    // The physical elevator should be all the way down when this is set.
    intakeExtensionMotor.setPosition(0);
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(Angle position) {
    intakeExtensionMotor.setControl(m_request.withPosition(position));
  }
}
