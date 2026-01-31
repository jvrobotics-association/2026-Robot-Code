// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
  private static final TalonFX shootermotor = new TalonFX(shooterConstants.MOTOR, "rio");
  final TalonFXConfiguration shootermotorConfig;
  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  /** Creates a new Shooter. */
  public Shooter() {
    // Create the configs used to configure the devices in this mechanism
    shootermotorConfig = new TalonFXConfiguration();
    shootermotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    shootermotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    shootermotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amp.of(20));

    // Apply the shootermotor config, retry config apply up to 5 times, report if failure
    StatusCode shootermotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shootermotorStatus = shootermotor.getConfigurator().apply(shootermotorConfig);
      if (shootermotorStatus.isOK()) break;
    }
    if (!shootermotorStatus.isOK()) {
      System.out.println(
          "Could not apply shooter motor config, error code: " + shootermotorStatus.toString());
    }
    // shootermotor.optimizeBusUtilization();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @AutoLogOutput(key = "Shooter/Shooter Speed")
  public void setShooterMotor(double speed) {
    shootermotor.setControl(m_manualRequest.withOutput(speed));
  }
}
