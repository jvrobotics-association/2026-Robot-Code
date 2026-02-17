// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class shooterConstants {
    public static final int LEFT_MOTOR = 30; // TODO: set left motor ID
    public static final int RIGHT_MOTOR = 40; // TODO: set right ID
      public static final double SPEED_MOE = 0.05;
  }

  public final class IntakeConstants {
    public static final int MOTOR = -1; // TODO: set motor ID
    public static final int ENCODER = -1; // TODO: set encoder ID
  }

  public final class shooterPitchConstants {
    public static final int MOTOR = -1; // TODO: set motor ID
    public static final int ENCODER = -1; // TODO: set encoder ID
    public static final double PITCH_MOE = 0.05;
  }

  public final class intakeExtensionConstants {
    public static final int MOTOR = -1; // TODO: set motor ID
    public static final int ENCODER = -1; // TODO: set encoder ID
  }

  public final class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);
    public static final Translation3d HUB_BLUE = 
        new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED = 
        new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final    Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4); // TODO: set funnel height
    public static final    Distance FUNNEL_RADIUS = Inches.of(24); // TODO: set funnel radius
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM = new Transform3d(
      new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18)), Rotation3d.kZero);
    public static final    Distance DISTANCE_ABOVE_FUNNEL =
        Inches.of(6); // TODO: set distance above funnel
  }

  public final class climberConstants {
    public static final int LEFT_MOTOR = -1; // TODO: set left motor ID
    public static final int LEFT_ENCODER = -1; // TODO: set encoder ID
    public static final int RIGHT_MOTOR = -1; // TODO: set right motor ID
    public static final int RIGHT_ENCODER = -1; // TODO: set encoder ID
    public static final String INTAKE_SPEED = null;
  }

  public final class controllerConstants {
    public static final double TRIGGER_THRESHOLD = 0.25;
  }
}
