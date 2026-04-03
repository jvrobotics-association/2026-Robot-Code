// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.LeftMotor.MOTOR_ID, "rio");
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.RightMotor.MOTOR_ID, "rio");
      
  /* Control Requests - Distinct objects for separate motor streams */
  private final MotionMagicVelocityTorqueCurrentFOC leftVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
  private final StrictFollower rightVelocityRequest =
      new StrictFollower(ShooterConstants.LeftMotor.MOTOR_ID);

  private double targetVelocityRPS = 0;

  private LoggedNetworkNumber LNN_RPS = new LoggedNetworkNumber("Shooter RPS", 0);

  /** Creates a new Shooter */
  public Shooter() {
    configureLeftMotor();
    configureRightMotor();
  }

  private void configureLeftMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.LeftMotor.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);
    
    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(45)
        .withSupplyCurrentLowerLimit(28)
        .withSupplyCurrentLowerTime(2);
    
    config.Voltage.withPeakForwardVoltage(12)
        .withPeakReverseVoltage(-12);

    config.Slot0.withKP(ShooterConstants.LeftMotor.PID_KP)
        .withKV(ShooterConstants.LeftMotor.PID_KV)
        .withKS(ShooterConstants.LeftMotor.PID_KS);

    config.MotionMagic.withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(ShooterConstants.LeftMotor.MM_ACCELERATION));
        //.withMotionMagicJerk(
         //   RotationsPerSecondPerSecond.per(Second).of(ShooterConstants.LeftMotor.MM_JERK));

    applyConfig(leftMotor, config);
  }

  private void configureRightMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    

    config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(ShooterConstants.RightMotor.SENSOR_TO_MECH);

    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.CurrentLimits.withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(45)
        .withSupplyCurrentLowerLimit(28)
        .withSupplyCurrentLowerTime(2);
    
    config.Voltage.withPeakForwardVoltage(12)
        .withPeakReverseVoltage(-12);

    config.Slot0.withKP(ShooterConstants.RightMotor.PID_KP)
        .withKV(ShooterConstants.RightMotor.PID_KV)
        .withKS(ShooterConstants.RightMotor.PID_KS);

    config.MotionMagic.withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(ShooterConstants.RightMotor.MM_ACCELERATION));
        //.withMotionMagicJerk(
        //    RotationsPerSecondPerSecond.per(Second).of(ShooterConstants.RightMotor.MM_JERK));

    applyConfig(rightMotor, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  public void shoot() {
    leftMotor.setControl(leftVelocityRequest.withVelocity(LNN_RPS.getAsDouble()));
  }

  public void stop() {
    leftMotor.stopMotor();
  }

  @Override
  public void periodic() {

    // AdvantageKit Logging (Crucial for the "Active" method to see what is happening)
    Logger.recordOutput("Shooter/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Shooter/LeftActualRPS", leftMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/RightActualRPS", rightMotor.getVelocity().getValueAsDouble());
  }

  // public boolean readyToShoot() {
  //   if (targetVelocityRPS <= 0) return false;

  //   double currentLeft = leftMotor.getVelocity().getValueAsDouble();
  //   double currentRight = rightMotor.getVelocity().getValueAsDouble();

  //   // Use a flat minimum floor for tolerance to avoid noise issues at low speeds
  //   double tolerance = Math.max(targetVelocityRPS * ShooterConstants.SPEED_MOE, 0.5);

  //   return Math.abs(currentLeft - targetVelocityRPS) <= tolerance
  //       && Math.abs(currentRight - targetVelocityRPS) <= tolerance;
  // }
}
