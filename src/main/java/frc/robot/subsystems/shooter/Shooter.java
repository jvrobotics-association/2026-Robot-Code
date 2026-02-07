// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
  /************ Declare Motors ************/
  private static final TalonFX shootermotor = new TalonFX(shooterConstants.MOTOR, "rio");
  private static final CANcoder encoder = new CANcoder(shooterConstants.ENCODER, "rio");
  
  /************ Declare Configs, Requests ************/
  final TalonFXConfiguration shootermotorConfig;
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(0);


  /** Creates a new Shooter */
  public Shooter() {
    /************ Configure Motors ************/
    shootermotorConfig = new TalonFXConfiguration();
    //Gets feed from either the encoder or the CANcoder
    shootermotorConfig
      .Feedback
      .withFeedbackRemoteSensorID(shooterConstants.ENCODER)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
      // set gearbox ratio to 1:1
      .withSensorToMechanismRatio(1)
      .withRotorToSensorRatio(1);
      // set Neutral Mode to Coast
    shootermotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    // Limits the amount of Amps the motor can draw
    // Volts * Amps = Watts
    shootermotorConfig
      .CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(Amps.of(20));

    //Configures the Cruise, Acceleration,Torque, and Stator limits
    shootermotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0; //TODO: set this value
    shootermotorConfig.MotionMagic.MotionMagicAcceleration = 0.0; //TODO: set this value
    shootermotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    shootermotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));
    
    // Apply the shooter motor config, retry config apply up to 5 times, report if failure
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = shootermotor.getConfigurator().apply(shootermotorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println("Could not apply shooter motor config, error code: " + motorStatus.toString());
    }

    /************ Configure Encoders ************/
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // Sets the speed and makes it a double
  public void setSpeed(double speed) {
    shootermotor.setControl(m_request.withVelocity(speed));
  }
}
