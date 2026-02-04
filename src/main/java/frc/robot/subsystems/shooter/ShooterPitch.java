// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import frc.robot.Constants.shooterPitchConstants;
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

public class ShooterPitch extends SubsystemBase {
   private static final TalonFX shooterPitchMotor = new TalonFX(shooterPitchConstants.MOTOR, "rio");
  private static final CANcoder encoder = new CANcoder(shooterPitchConstants.ENCODER, "rio");
  final TalonFXConfiguration shooterPitchMotorConfig;
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  final public MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  /** Creates a new ShooterPitch. */
  public ShooterPitch() {
    shooterPitchMotorConfig = new TalonFXConfiguration();
    shooterPitchMotorConfig
    .Feedback
    .withFeedbackRemoteSensorID(shooterPitchConstants.ENCODER)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withSensorToMechanismRatio(1)
    .withRotorToSensorRatio(1);
    shooterPitchMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    shooterPitchMotorConfig
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
    // Apply the shootermotor config, retry config apply up to 5 times, report if failure
    StatusCode shooterPitchMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterPitchMotorStatus = shooterPitchMotor.getConfigurator().apply(shooterPitchMotorConfig);
      if (shooterPitchMotorStatus.isOK()) break;
    }
    if (!shooterPitchMotorStatus.isOK()) {
      System.out.println(
          "Could not apply shooter pitch motor config, error code: " + shooterPitchMotorStatus.toString());
    }

    shooterPitchMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    shooterPitchMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    shooterPitchMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    shooterPitchMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      motorStatus = shooterPitchMotor.getConfigurator().apply(shooterPitchMotorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println(
          "Could not apply shooter pitch motor config, error code: " + motorStatus.toString());
    }
    
    // Reset the position that the elevator currently is at to 0.
    // The physical elevator should be all the way down when this is set.
    shooterPitchMotor.setPosition(0);
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(Angle position) {
    shooterPitchMotor.setControl(m_request.withPosition(position));
  }
  
}
