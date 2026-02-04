// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR, "rio");
  private static final CANcoder encoder = new CANcoder(IntakeConstants.ENCODER, "rio");
  final TalonFXConfiguration intakeMotorConfig;
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

  public Intake() {
    intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig
    .Feedback
    .withFeedbackRemoteSensorID(IntakeConstants.ENCODER)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withSensorToMechanismRatio(1) // TODO: Set Sensor to Mechanism Ratio
    .withRotorToSensorRatio(1); //TODO: Set Rotor to Sensor Ratio
    intakeMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));
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

    intakeMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0; //TODO: Set Cruise Velocity
    intakeMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; //TODO: Set Acceleration
    intakeMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    intakeMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = intakeMotor.getConfigurator().apply(intakeMotorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println(
          "Could not apply intake motor config, error code: " + motorStatus.toString());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    intakeMotor.setControl(m_request.withVelocity(speed));
  }
}
