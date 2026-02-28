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

  public final class IndexerConstants {
    public static final int INDEXER_MOTOR = -1;
    public static final double INDEXER_SPEED = 0.5; // TODO: set speed

    public static final int SENSOR_TO_MECH_RATIO = 1; // TODO: set ratio
    public static final int STATOR_AMP_LIMIT = 20;
    public static final int FORWARD_TORQUE_AMPS_LIMIT = 40;

    public static final double MM_ACCELERATION = 0.0;
    public static final double MM_JERK = 0.0;

    public static final double PID_KS = 0.0;
    public static final double PID_KV = 0.0;
    public static final double PID_KA = 0.0;
    public static final double PID_KP = 0.0;
    public static final double PID_KI = 0.0;
    public static final double PID_KD = 0.0;
  }

  public final class ShooterConstants {
    public final class LeftMotor {
      public static final int CAN_ID = 30;
      public static final double PID_KS = 0.0;
      public static final double PID_KV = 0.0;
      public static final double PID_KA = 0.0;
      public static final double PID_KP = 0.0;
      public static final double PID_KI = 0.0;
      public static final double PID_KD = 0.0;

      public static final double SENSOR_TO_MECH = 1;

      public static final double STATOR_AMP_LIMIT = 20;
      public static final double PEAK_FORWARD_TORQUE_CURRENT = 40;

      public static final double MM_ACCELERATION = 10;
      public static final double MM_JERK = 100;
    }

    public final class RightMotor {
      public static final int CAN_ID = 40;
      public static final double PID_KS = 0.0;
      public static final double PID_KV = 0.0;
      public static final double PID_KA = 0.0;
      public static final double PID_KP = 0.0;
      public static final double PID_KI = 0.0;
      public static final double PID_KD = 0.0;

      public static final double SENSOR_TO_MECH = 1;

      public static final double STATOR_AMP_LIMIT = 20;
      public static final double PEAK_FORWARD_TORQUE_CURRENT = 40;

      public static final double MM_ACCELERATION = 10;
      public static final double MM_JERK = 100;
    }

    public static final double SPEED_MOE = 0.05;
    public static final double FLYWHEEL_CIRCUMFERENCE = 3.0; // TODO: Verify, Inches
  }

  public final class IntakeConstants {
    public static final int MOTOR = 20;
    public static final String INTAKE_SPEED = null;
    public static final int STATOR_AMP_LIMIT = 20;
    public static final int SENSOR_TO_MECH_RATIO = -1; // TODO: set ratio
    public static final int MM_VEL_TORQUE_CURRENT_FOC = -1; // TODO: set current
    public static final int PEAK_FORWARD_TORQUE_CURRENT = 40;
  }

  public final class ShooterPitchConstants {
    public static final int CAN_ID = 10;
    public static final double PITCH_MOE = 0.05;
    public static final double SENSOR_TO_MECH = 1;
    public static final double STATOR_AMP_LIMIT = 20;
    public static final double PEAK_FORWARD_TORQUE_CURRENT = 40;
    public static final double PEAK_REVERSE_TORQUE_CURRENT = -40;
    public static final double MM_ACCELERATION = 5.0;
    public static final double MM_CRUISE_VEL = 10.0;
    public static final double MM_JERK = 0.0;
    public static final double PID_KS = 0.0;
    public static final double PID_KV = 0.0;
    public static final double PID_KA = 0.0;
    public static final double PID_KP = 0.0;
    public static final double PID_KI = 0.0;
    public static final double PID_KD = 0.0;
  }

  public final class IntakeExtensionConstants {
    public static final int MOTOR = -1; // TODO: set motor ID
    public static final int ENCODER = -1; // TODO: set encoder ID
  }

  public final class ClimberConstants {
    public static final int LEFT_MOTOR = -1; // TODO: set left motor ID
    public static final int LEFT_ENCODER = -1; // TODO: set encoder ID
    public static final int RIGHT_MOTOR = -1; // TODO: set right motor ID
    public static final int RIGHT_ENCODER = -1; // TODO: set encoder ID
    public static final int CLIMBER_POSITION = -1; // TODO: set position
    public static final int MM_ACCELERATION = -1; // TODO: set value
    public static final int MM_CRUISE_VEL = -1; // TODO: set value
    public static final int STATOR_AMPS = 20;
    public static final int STATOR_CURRENT_LIMIT = 50;
    public static final int FORWARD_TORQUE_AMPS_LIMIT = 40;
    public static final int SENSOR_TO_MECH_RATIO = -1; // TODO: set ratio
  }

  public final class MidSystemConstants {
    public static final double CALCS_PER_SECOND = 10.0; // TODO: Verify
    public static final double AIM_TOLERANCE_DEGREES = 3.0;
  }

  public final class RobotConstants {
    public static final Distance FULL_WIDTH = Inches.of(25); // TODO: Verify // Frame plus bumper
    public static final Distance FULL_LENGTH = Inches.of(30); // TODO: Verify // Frame plus bumper
  }

  public final class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);
    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static final Translation3d HUB_BLUE =
        new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED =
        new Translation3d(
            FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));

    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4); // TODO: set funnel height
    public static final Distance FUNNEL_RADIUS = Inches.of(24); // TODO: set funnel radius
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18)), Rotation3d.kZero);
    public static final Distance DISTANCE_ABOVE_FUNNEL =
        Inches.of(6); // TODO: set distance above funnel
  }

  public final class ControllerConstants {
    public static final double TRIGGER_THRESHOLD = 0.25;
  }
}
