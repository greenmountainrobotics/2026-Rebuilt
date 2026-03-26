// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonFX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.feed.Feed;
import frc.robot.subsystems.feed.FeedIO;
import frc.robot.subsystems.feed.FeedIOSim;
import frc.robot.subsystems.feed.FeedIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Feed feed;
  private final Shooter shooter;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Shuffleboard pose target (for "Drive to Pose" button)
  private final GenericEntry targetPoseX;
  private final GenericEntry targetPoseY;
  private final GenericEntry targetPoseDegrees;

  // Shuffleboard AprilTag align (tag ID 1–32)
  private final GenericEntry alignTagIdEntry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonFX(), new GyroIOPigeon2());
        intake = new Intake(new IntakeIOSparkMax());
        feed = new Feed(new FeedIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        intake = new Intake(new IntakeIOSim());
        feed = new Feed(new FeedIOSim());
        shooter = new Shooter(new ShooterIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        intake = new Intake(new IntakeIO() {});
        feed = new Feed(new FeedIO() {});
        shooter = new Shooter(new ShooterIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    AutoFactory choreoAutoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);

    // BlueMiddle has a split: run both segments so the robot reaches the end pose (~2.53s total).
    autoChooser.addOption(
        "BlueMiddle",
        Commands.runOnce(drive::resetChoreoSpeeds)
            .andThen(choreoAutoFactory.resetOdometry("BlueMiddle", 0))
            .andThen(choreoAutoFactory.trajectoryCmd("BlueMiddle", 0))
            .andThen(Commands.runOnce(drive::resetChoreoSpeeds))
            .andThen(choreoAutoFactory.trajectoryCmd("BlueMiddle", 1)));

    autoChooser.addOption(
        "Forward",
        Commands.runOnce(drive::resetChoreoSpeeds)
            .andThen(choreoAutoFactory.resetOdometry("Forward"))
            .andThen(choreoAutoFactory.trajectoryCmd("Forward")));

    // Example "between trajectories" auto:
    // - Reset odometry to the start of the first path
    // - Follow first trajectory while slowly feeding
    // - After first path, spin up shooter for a moment
    // - Follow second trajectory while feeding harder
    // - Finally, stop shooter + feed
    autoChooser.addOption(
        "Choreo: Two Paths + Feed/Shoot Example",
        Commands.sequence(
            Commands.runOnce(drive::resetChoreoSpeeds),
            choreoAutoFactory.resetOdometry("BlueMiddle"),
            Commands.deadline(
                choreoAutoFactory.trajectoryCmd("BlueMiddle"),
                feed.runAtVoltage(4.0).withName("Feed during first path")),
            Commands.sequence(
                shooter
                    .runAtVelocity(ShooterConstants.shootVelocityRadPerSec * 0.8)
                    .withTimeout(1.5)
                    .withName("Spin up shooter"),
                Commands.waitSeconds(0.25)),
            Commands.runOnce(drive::resetChoreoSpeeds),
            Commands.deadline(
                choreoAutoFactory.trajectoryCmd("myAutoPart2"),
                feed.runAtVoltage(8.0).withName("Feed during second path")),
            Commands.parallel(
                Commands.runOnce(feed::stop, feed), Commands.runOnce(shooter::stop, shooter))));

    autoChooser.addOption(
        "Test Pose PID",
        DriveCommands.driveToPose(
            drive, () -> new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(90)), true));

    // Set up SysId routines
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

    // setting tele target pose
    ShuffleboardTab poseTab = Shuffleboard.getTab("Pose");
    targetPoseX =
        poseTab
            .add("Target X (m)", 2.5)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -10, "max", 10))
            .getEntry();
    targetPoseY =
        poseTab
            .add("Target Y (m)", 4.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -10, "max", 10))
            .getEntry();
    targetPoseDegrees =
        poseTab
            .add("Target Heading (deg)", 90.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
    poseTab.add(
        "Drive to Pose",
        DriveCommands.driveToPose(drive, this::getTargetPoseFromShuffleboard, true));

    alignTagIdEntry =
        poseTab
            .add("Align to AprilTag ID", 5)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 32))
            .getEntry();
    poseTab.add(
        "Align to AprilTag",
        DriveCommands.driveToPose(
            drive,
            () ->
                getTargetPoseFromAprilTag((int) alignTagIdEntry.getDouble(5))
                    .orElse(drive.getPose()),
            true));

    // Camera output for SmartDashboard/Shuffleboard using PhotonVision cameras
    // Adjust the host/port below if your PhotonVision instance uses a different address.
    ShuffleboardTab camerasTab = Shuffleboard.getTab("Cameras");
    HttpCamera cam1 =
        new HttpCamera(
            camera0Name,
            "http://photonvision.local:1181/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);
    HttpCamera cam2 =
        new HttpCamera(
            camera1Name,
            "http://photonvision.local:1182/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);
    camerasTab.add("Cam 1", cam1).withWidget(BuiltInWidgets.kCameraStream);
    camerasTab.add("Cam 2", cam2).withWidget(BuiltInWidgets.kCameraStream);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  /** Returns the target pose from Shuffleboard (Pose tab). */
  private Pose2d getTargetPoseFromShuffleboard() {
    return new Pose2d(
        targetPoseX.getDouble(2.5),
        targetPoseY.getDouble(4.0),
        Rotation2d.fromDegrees(targetPoseDegrees.getDouble(90.0)));
  }

  /**
   * Target pose to align in front of an AprilTag (offset so we don't drive into the tag). Uses
   * VisionConstants.alignToTagOffsetXMeters and alignToTagOffsetYMeters in the tag's frame. Returns
   * empty if the tag ID is not in the layout.
   */
  private Optional<Pose2d> getTargetPoseFromAprilTag(int tagId) {
    var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
    if (tagPoseOpt.isEmpty()) return Optional.empty();

    Pose3d tag = tagPoseOpt.get();
    Translation2d tagPos = tag.getTranslation().toTranslation2d();
    Rotation2d tagHeading = tag.getRotation().toRotation2d();
    // Offset in tag frame: +X = forward from tag, +Y = left; then rotate to field frame
    Translation2d offsetInTagFrame =
        new Translation2d(alignToTagOffsetXMeters, alignToTagOffsetYMeters);
    Translation2d worldOffset = offsetInTagFrame.rotateBy(tagHeading);
    return Optional.of(new Pose2d(tagPos.plus(worldOffset), tagHeading));
  }

  private void configureButtonBindings() {
    // Left trigger: proportional brake (harder press = stronger brake), 0–1 from axis
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> controller.getRightX(),
            () -> controller.getLeftTriggerAxis()));

    controller.a().onTrue(Commands.runOnce(() -> drive.setTurbo(!drive.getTurbo()), drive));

    // Intake: right bumper runs intake while held (full forward voltage)
    controller.rightBumper().whileTrue(intake.runWithVoltage(() -> 12.0).withName("Intake"));

    // Shooter + Feed: right trigger runs both at same time while held
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.runAtVelocity(ShooterConstants.shootVelocityRadPerSec).withName("Shooter"),
                feed.runAtVoltage(12.0).withName("Feed")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
