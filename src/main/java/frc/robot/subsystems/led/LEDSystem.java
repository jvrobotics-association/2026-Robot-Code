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

public class LEDSystem extends SubsystemBase {
  private final CANdle ledController = new CANdle(4, "rio");

  /** Creates a new LEDSubsystem. */
  public LEDSystem() {
    CANdleConfiguration config = new CANdleConfiguration();

    config.LED.withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0);
    config.CANdleFeatures.withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled);

    applyConfig(config);

    ledController.clearAllAnimations();

    ledController.optimizeBusUtilization();
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
              .withFrameRate(30));
      case FlowDirectionRedInverted:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
              .withSlot(slot)
              .withDirection(AnimationDirectionValue.Backward)
              .withColor(new RGBWColor(Color.kRed))
              .withFrameRate(30));
      case FlowDirectionBlue:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
              .withSlot(slot)
              .withDirection(AnimationDirectionValue.Forward)
              .withColor(new RGBWColor(Color.kBlue))
              .withFrameRate(30));
      case FlowDirectionBlueInverted:
        ledController.setControl(
            new ColorFlowAnimation(indexStart, indexStop)
              .withSlot(slot)
              .withDirection(AnimationDirectionValue.Backward)
              .withColor(new RGBWColor(Color.kBlue))
              .withFrameRate(30));
    }
  }

  public void setRightShooterBar(AnimationType animationType) {
    // TODO: Determine exact LED index for Right Shooter Bar
    applyRequest(8, 15, 1, animationType);
  }

  public void setRightHopperBar(AnimationType animationType) {
    // TODO: Determine exact LED index for Right Hopper Bar
    applyRequest(16, 20, 2, animationType);
  }

  public void setLeftHopperBar(AnimationType animationType) {
    // TODO: Determine exact LED index for Left Hopper Bar
    applyRequest(21, 25, 3, animationType);
  }

  public void setLeftShooterBar(AnimationType animationType) {
    // TODO: Determine exact LED index for Left Shooter Bar
    applyRequest(26, 30, 4, animationType);
  }

  public void setEntireRightSide(AnimationType animationType) {
    // TODO: Update the index range with the combined index range of Right Shooter and Hopper bars
    applyRequest(8, 20, 5, animationType);
  }

  public void setEntireLeftSide(AnimationType animationType) {
    // TODO: Update the index range with the combined index range of Left Shooter and Hopper bars
    applyRequest(21, 30, 6, animationType);
  }

  public void setAll(AnimationType animationType) {
    // TODO: Update the index range with the combined index range of all LEDs
    applyRequest(8, 30, 6, animationType);
  }
}
