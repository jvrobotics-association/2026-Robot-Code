// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterPitchConstants;

public class ShooterPitch extends SubsystemBase {
  /** Creates and Declares a TalonFX motor and a CANcoder */
  private static final TalonFX shooterPitchMotor = new TalonFX(shooterPitchConstants.MOTOR, "rio");

  private static final CANcoder encoder = new CANcoder(shooterPitchConstants.ENCODER, "rio");

  final TalonFXConfiguration shooterPitchMotorConfig;
  public final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
  
  private double setPointPosition;


  /** Creates a new ShooterPitch */
  public ShooterPitch() {
    shooterPitchMotorConfig = new TalonFXConfiguration();
    // Gets feedback from either the encoder or the CANcoder
    shooterPitchMotorConfig
        .Feedback
        .withFeedbackRemoteSensorID(shooterPitchConstants.ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1); // TODO: Set Sensor to Mechanism Ratio

    // Sets the Neutral Mode to Brake
    shooterPitchMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    shooterPitchMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(20));
    // Configures the Cruise, Acceleration, Torque, and Stator limits
    shooterPitchMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        0.0; // TODO: Set Cruise Velocity
    shooterPitchMotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; // TODO: Set Acceleration
    shooterPitchMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    shooterPitchMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Apply the motor config, retry config apply up to 5 times, report if failure
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = shooterPitchMotor.getConfigurator().apply(shooterPitchMotorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println(
          "Could not apply shooter pitch motor config, error code: " + motorStatus.toString());
    }

    /** Configure Encoders */
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
    shooterPitchMotor.setPosition(0);
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    shooterPitchMotor.setControl(m_request.withPosition(setPointPosition));
  }

  public void setAngle(double position) {
    this.setPointPosition = position; // TODO: Limit updates to a margin of error (i.e. 2%)
  }

  public boolean readyToShoot(){
    return (shooterPitchMotor.getPosition().getValueAsDouble() >= ((1.00-shooterPitchConstants.PITCH_MOE))*setPointPosition) && 
      (shooterPitchMotor.getPosition().getValueAsDouble() <= ((1.00+shooterPitchConstants.PITCH_MOE)*setPointPosition));
  }


}
