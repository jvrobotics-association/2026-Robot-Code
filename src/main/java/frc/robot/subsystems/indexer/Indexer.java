// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  final MotionMagicConfigs indexerMotorMMConfigs;
  final Slot0Configs indexerMotorSlot0;  
  final TalonFXConfiguration indexerMotorConfig;

  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  private static final TalonFX indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR, "rio");

  private final LoggedNetworkNumber ShooterSpeed = new LoggedNetworkNumber("Shooter Speed", 0.0);

  public double indexerSpeed = 0;

  /** Creates a new Indexer. */
  public Indexer() {

    indexerMotorConfig = new TalonFXConfiguration();

    indexerMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .SensorToMechanismRatio = 1; // TODO: Verify if this or IntakeMotor's config works better
    
    indexerMotorConfig
      .MotorOutput
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.Clockwise_Positive);

    indexerMotorConfig
      .CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(Amps.of(20));
    
    indexerMotorSlot0 = indexerMotorConfig.Slot0;
    indexerMotorSlot0.kS = 0.0; // Add 0.25 V output to overcome static friction
    indexerMotorSlot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    indexerMotorSlot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    indexerMotorSlot0.kP = 0.0; // A position error of 0.2 rotations results in 12 V output
    indexerMotorSlot0.kI = 0.0; // No output for integrated error
    indexerMotorSlot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output

    indexerMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    indexerMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    indexerMotorMMConfigs = indexerMotorConfig.MotionMagic;
    indexerMotorMMConfigs
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    // Apply the left shooter motor config, retry config apply up to 5 times, report if failure
    StatusCode indexerMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      indexerMotorStatus = indexerMotor.getConfigurator().apply(indexerMotorConfig);
      if (indexerMotorStatus.isOK()) break;
    }
    if (!indexerMotorStatus.isOK()) {
      System.out.println(
        "Could not apply indexer shooter motor config, error code: " + indexerMotorStatus.toString());
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        indexerMotor.setControl(m_manualRequest.withOutput(indexerSpeed));

  }

public void setspeed(double speed){
  this.indexerSpeed = speed;

}

}
