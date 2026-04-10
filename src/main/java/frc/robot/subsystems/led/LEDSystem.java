// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDSystem extends SubsystemBase {
  private final CANdle ledController = new CANdle(4, "rio");

  /** Creates a new LEDSubsystem. */
  public LEDSystem() {
    CANdleConfiguration config = new CANdleConfiguration();

    config.LED.withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0);
    config.CANdleFeatures.withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled);

    applyConfig(config);

    for (int i = 0; i < 8; ++i) {
      ledController.setControl(new EmptyAnimation(i));
    }
  }

  private void applyConfig(CANdleConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = ledController.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  private void applyRequest(int indexStart, int indexStop, int slot, AnimationType animationType) {
    switch (animationType) {
      default:
        ledController.setControl(new EmptyAnimation(slot));
        ledController.setControl(new SolidColor(indexStart, indexStop).withColor(new RGBWColor()));
        break;
      case Off:
        ledController.setControl(new EmptyAnimation(slot));
        ledController.setControl(new SolidColor(indexStart, indexStop).withColor(new RGBWColor()));
        break;
      case SolidGreen:
        ledController.setControl(
            new SolidColor(indexStart, indexStop).withColor(new RGBWColor(Color.kGreen)));
        break;
      case SolidBlue:
        ledController.setControl(
            new SolidColor(indexStart, indexStop).withColor(new RGBWColor(Color.kBlue)));
        break;
      case SolidRed:
        ledController.setControl(
            new SolidColor(indexStart, indexStop).withColor(new RGBWColor(Color.kRed)));
        break;
      case StrobeGreen:
        ledController.setControl(
            new StrobeAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withColor(new RGBWColor(Color.kGreen))
                .withFrameRate(10));
      case StrobeBlue:
        ledController.setControl(
            new StrobeAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withColor(new RGBWColor(Color.kBlue))
                .withFrameRate(10));
      case StrobeRed:
        ledController.setControl(
            new StrobeAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withColor(new RGBWColor(Color.kRed))
                .withFrameRate(10));
      case Rainbow:
        ledController.setControl(new RainbowAnimation(indexStart, indexStop).withSlot(slot));
      case FlowDirectionRed:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withDirection(AnimationDirectionValue.Forward)
                .withColor(new RGBWColor(Color.kRed))
                .withFrameRate(100));
      case FlowDirectionRedInverted:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withDirection(AnimationDirectionValue.Backward)
                .withColor(new RGBWColor(Color.kRed))
                .withFrameRate(100));
      case FlowDirectionBlue:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withDirection(AnimationDirectionValue.Forward)
                .withColor(new RGBWColor(Color.kBlue))
                .withFrameRate(100));
      case FlowDirectionBlueInverted:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
                .withSlot(slot)
                .withDirection(AnimationDirectionValue.Backward)
                .withColor(new RGBWColor(Color.kBlue))
                .withFrameRate(1000));
    }
  }

  public void setRightShooterBar(AnimationType animationType) {
    applyRequest(8, 26, 0, animationType);
  }

  public void setRightHopperBar(AnimationType animationType) {
    applyRequest(27, 49, 0, animationType);
  }

  public void setLeftHopperBar(AnimationType animationType) {
    applyRequest(50, 72, 0, animationType);
  }

  public void setLeftShooterBar(AnimationType animationType) {
    applyRequest(73, 91, 0, animationType);
  }

  public void setEntireRightSide(AnimationType animationType) {
    applyRequest(8, 49, 0, animationType);
  }

  public void setEntireLeftSide(AnimationType animationType) {
    applyRequest(50, 91, 0, animationType);
  }

  public void setAll(AnimationType animationType) {
    applyRequest(8, 91, 0, AnimationType.Rainbow);
  }

  public void setRedSolid() {
    ledController.setControl(new SolidColor(8, 91).withColor(new RGBWColor(Color.kRed)));
  }

  public void setBlueSolid() {
    ledController.setControl(new SolidColor(8, 91).withColor(new RGBWColor(Color.kBlue)));
  }

  public void setRainbowAll() {
    ledController.setControl(new RainbowAnimation(8, 91).withSlot(0));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED System/Applied Control", ledController.getAppliedControl().getName());
  }
}
