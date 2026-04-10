// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AdvancedShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Tower;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.subsystems.led.LEDSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPitch;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final ShooterPitch pitch;
  private final Indexer indexer;
  private final Tower tower;
  private final Intake intake;
  private final IntakeExtension intakeExt;
  private final Hopper hopper;
  private final LEDSystem ledSystem;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorPanel = new CommandXboxController(1);

  private Alliance alliance;
  private Translation2d hubTarget;
  private Command hopperExtendCommand;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    updateHubTarget();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        pitch = new ShooterPitch();
        shooter = new Shooter();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();
        ledSystem = new LEDSystem();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        pitch = new ShooterPitch();
        shooter = new Shooter();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();
        ledSystem = new LEDSystem();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter();
        pitch = new ShooterPitch();
        indexer = new Indexer();
        tower = new Tower();
        intake = new Intake();
        intakeExt = new IntakeExtension();
        hopper = new Hopper();
        ledSystem = new LEDSystem();

        // Pass nulls or empty shells for Replay mode depending on your AdvantageKit setup
        // midSystem = new MidSystem(null, null, null, null, null, null, null, null, null, null);
        break;
    }

    NamedCommands.registerCommand(
        "extendHopperIntake",
        Commands.sequence(
            Commands.runOnce(hopper::deploy, hopper),
            Commands.waitSeconds(1.5),
            Commands.runOnce(intakeExt::deploy, intakeExt),
            Commands.waitSeconds(1),
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(intakeExt::stop, intakeExt)));

    NamedCommands.registerCommand(
        "runIntake", Commands.runOnce(() -> intake.runIntake(0.9), intake));

    NamedCommands.registerCommand("stopIntake", Commands.runOnce(intake::stopIntake, intake));

    NamedCommands.registerCommand(
        "basicShoot",
        Commands.parallel(
            Commands.runEnd(shooter::staticShoot, shooter::stop, shooter),
            Commands.sequence(
                Commands.waitSeconds(1.5).until(shooter::atTargetVelocity),
                Commands.parallel(
                    Commands.runEnd(tower::start, tower::stop, tower),
                    Commands.runEnd(indexer::feed, indexer::stop, indexer)))));

    NamedCommands.registerCommand(
        "bumpBalls",
        Commands.sequence(
                Commands.waitSeconds(2),
                Commands.runOnce(() -> intake.runIntake(0.75)),
                Commands.runOnce(intakeExt::bumpRetract, intakeExt),
                Commands.waitSeconds(2),
                Commands.runOnce(intakeExt::fullRetract, intakeExt),
                Commands.repeatingSequence(
                    Commands.waitSeconds(0.75),
                    Commands.runOnce(intakeExt::fullRetract, intakeExt),
                    Commands.waitSeconds(0.75),
                    Commands.runOnce(intakeExt::deploy, intakeExt)))
            .finallyDo(
                () -> {
                  intake.stopIntake();
                  intakeExt.deploy();
                }));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    configureBindings();
    disabledLedState();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureBindings() {
    /////////////////////////////////
    //////// COMMON COMMANDS ////////
    /////////////////////////////////

    // Runs the intake, if it is already running it cancels the incoming command to prevent one
    // driver from stopping it on the other
    Command runIntake =
        Commands.startEnd(() -> intake.runIntake(0.9), intake::stopIntake, intake)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    // Locks the wheels to an X pattern to make the robot harder to move
    Command xLockWheelsCommand = Commands.runOnce(drive::stopWithX, drive);

    ///////////////////////////////////
    //////// CONTROLLER INPUTS ////////
    ///////////////////////////////////

    // Default for the drive command to field oriented drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Runs the intake to intake fuel
    controller.leftTrigger(ControllerConstants.TRIGGER_THRESHOLD).whileTrue(runIntake);

    // controller
    //     .x()
    //     .toggleOnTrue(Commands.runOnce(() -> ledSystem.setAll(AnimationType.Rainbow),
    // ledSystem));

    // Aim the robot at the hub, this is mainly used for backup if the main shoot command is not
    // working
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    new Rotation2d(
                            hubTarget.getX() - drive.getPose().getX(),
                            hubTarget.getY() - drive.getPose().getY())
                        .plus(Rotation2d.fromDegrees(180.0))));

    // Run the shoot command that automatically aligns the robot and shoots when ready
    controller
        .rightTrigger(ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () ->
                        new Rotation2d(
                                hubTarget.getX() - drive.getPose().getX(),
                                hubTarget.getY() - drive.getPose().getY())
                            .plus(Rotation2d.fromDegrees(180.0))),
                new AdvancedShootCommand(
                    shooter,
                    () -> operatorPanel.rightBumper().getAsBoolean(),
                    pitch,
                    tower,
                    indexer,
                    ledSystem,
                    drive::getPose,
                    () -> hubTarget,
                    () -> alliance)));

    // Drive with the robot heading locked straight, it will snap to whichever side of the robot is
    // already closest to that heading
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () ->
    //                 Math.abs(drive.getRotation().minus(Rotation2d.fromDegrees(0.0)).getDegrees())
    //                         <= Math.abs(
    //                             drive
    //                                 .getRotation()
    //                                 .minus(Rotation2d.fromDegrees(180.0))
    //                                 .getDegrees())
    //                     ? Rotation2d.fromDegrees(0.0)
    //                     : Rotation2d.fromDegrees(180.0)));

    /* Drive with the robot heading locked straight, this will make sure the shooter is pointed in
    the direction we need to shoot. This may cause the robot to spin more than expected if it is not already somewhat facing that direction
    */
    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    alliance == Alliance.Blue
                        ? Rotation2d.fromDegrees(180.0)
                        : Rotation2d.fromDegrees(0.0)));

    // Manually pull the hopper back in when the zeroing is incorrect
    controller
        .button(7)
        .whileTrue(Commands.runEnd(() -> hopper.setManualDutyCycle(-0.1), hopper::stop, hopper));

    // Resets the robots Zero heading to the current orientation of the robot
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    ///////////////////////////////////////
    //////// OPERATOR PANEL INPUTS ////////
    ///////////////////////////////////////

    // Hopper In
    operatorPanel
        .povRight()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(intakeExt::fullRetract, intakeExt),
                Commands.waitSeconds(0.75),
                Commands.runOnce(intakeExt::stop, intakeExt),
                Commands.runOnce(hopper::retract, hopper),
                Commands.waitSeconds(5),
                Commands.runOnce(hopper::stop, hopper)));

    // Hopper out
    operatorPanel
        .povLeft()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(hopper::deploy, hopper),
                Commands.waitSeconds(1.5),
                Commands.runOnce(intakeExt::deploy, intakeExt),
                Commands.waitSeconds(1),
                Commands.runOnce(hopper::stop, hopper),
                Commands.runOnce(intakeExt::stop, intakeExt)));

    // Run the intake
    operatorPanel.b().whileTrue(runIntake);

    // Manually outake fuel by running the indexer and intake in reverse
    operatorPanel
        .y()
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(() -> intake.runIntake(-0.5), intake::stopIntake, intake),
                Commands.startEnd(indexer::reverseFeed, indexer::stop, indexer)));

    // Raise the intake arm so that balls in the front of the hopper are moved to the back
    operatorPanel
        .a()
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(() -> intake.runIntake(0.75)),
                    Commands.runOnce(intakeExt::bumpRetract, intakeExt),
                    Commands.waitSeconds(2),
                    Commands.runOnce(intakeExt::fullRetract, intakeExt),
                    Commands.repeatingSequence(
                        Commands.waitSeconds(0.75),
                        Commands.runOnce(intakeExt::fullRetract, intakeExt),
                        Commands.waitSeconds(0.75),
                        Commands.runOnce(intakeExt::deploy, intakeExt)))
                .finallyDo(
                    () -> {
                      intake.stopIntake();
                      intakeExt.deploy();
                    }));

    // Full intake extension retract
    operatorPanel
        .rightTrigger(ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(() -> intake.runIntake(0.75)),
                    Commands.runOnce(intakeExt::fullRetract, intakeExt),
                    Commands.waitSeconds(0.75),
                    Commands.runOnce(intakeExt::deploy, intakeExt))
                .finallyDo(
                    () -> {
                      intake.stopIntake();
                      intakeExt.deploy();
                    }));

    // Bump intake extension retract
    operatorPanel
        .leftTrigger(ControllerConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(() -> intake.runIntake(0.75)),
                    Commands.runOnce(intakeExt::bumpRetract, intakeExt),
                    Commands.waitSeconds(0.75),
                    Commands.runOnce(intakeExt::deploy, intakeExt))
                .finallyDo(
                    () -> {
                      intake.stopIntake();
                      intakeExt.deploy();
                    }));

    // Simple shoot command in the event the fancy one does not work
    operatorPanel
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(shooter::staticShoot, shooter::stop, shooter),
                Commands.sequence(
                    Commands.waitSeconds(1.5).until(shooter::atTargetVelocity),
                    Commands.parallel(
                        Commands.runEnd(tower::start, tower::stop, tower),
                        Commands.runEnd(indexer::feed, indexer::stop, indexer)))));

    // Lock the drive modules to an X configuration to help avoid getting bumped around
    operatorPanel.x().whileTrue(xLockWheelsCommand);
  }

  public void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /**
   * Triggers a refresh of the hub target by checking which alliance we are on and setting the
   * appropriate location
   */
  public void updateHubTarget() {
    updateAlliance();
    hubTarget = alliance == Alliance.Blue ? FieldConstants.HUB_BLUE : FieldConstants.HUB_RED;
  }

  /** Schedules the hopperExtendCommand to extend the hopper and intake arm */
  public void extendHopper() {
    CommandScheduler.getInstance().schedule(hopperExtendCommand);
  }

  public void disabledLedState() {
    ledSystem.setRainbowAll();
  }

  public void enabledLedState() {
    updateAlliance();
    if (alliance == Alliance.Blue) {
      ledSystem.setBlueSolid();
    } else ledSystem.setRedSolid();
  }
}
