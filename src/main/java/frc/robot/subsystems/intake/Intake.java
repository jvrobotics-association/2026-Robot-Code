// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  /** Declares the Motor */
  private static final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR, "rio");

  /** Declares Configs */
  final TalonFXConfiguration intakeMotorConfig;

  final MotionMagicConfigs intakeMotorMMConfigs;
  final Slot0Configs intakeMotorSlot0;

  /** Motor Control Requests */
  final MotionMagicVelocityTorqueCurrentFOC m_request = new MotionMagicVelocityTorqueCurrentFOC(IntakeConstants.MM_VEL_TORQUE_CURRENT_FOC);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  /************ Class Member Variables ************/
  private final LoggedNetworkNumber IntakeSpeed = new LoggedNetworkNumber("Intake Speed", 0.0);

  /** Creates a new Intake */
  public Intake() {

    /** Configures the Motor and sets up the Feedback. */
    intakeMotorConfig = new TalonFXConfiguration();
    // Get feedback from either the encoder or the CANcoder
    intakeMotorConfig
        .Feedback
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(IntakeConstants.SENSOR_TO_MECH_RATIO); // TODO: Verify if this or ShooterMotor's config works better
    // Sets Neutral mode to coast
    intakeMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    // Sets the current limits for the
    intakeMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_AMP_LIMIT));

    intakeMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(IntakeConstants.PEAK_FORWARD_TORQUE_CURRENT));
    intakeMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_CURRENT_LIMIT));

    intakeMotorSlot0 = intakeMotorConfig.Slot0;
    intakeMotorSlot0.kS = 0.0; // Add 0.25 V output to overcome static friction
    intakeMotorSlot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    intakeMotorSlot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    intakeMotorSlot0.kP = 0.0; // A position error of 0.2 rotations results in 12 V output
    intakeMotorSlot0.kI = 0.0; // No output for integrated error
    intakeMotorSlot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output

    intakeMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40));
    intakeMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Configures the Acceleration, Jerk
    intakeMotorMMConfigs = intakeMotorConfig.MotionMagic;
    intakeMotorMMConfigs
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

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

  public void startIntake() {
    intakeMotor.setControl(m_manualRequest.withOutput(IntakeSpeed.getAsDouble()));
  }

  public void stopIntake() {
    intakeMotor.setControl(m_manualRequest.withOutput(0.0));
  }
}
