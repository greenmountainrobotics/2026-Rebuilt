package frc.robot.subsystems.launch;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.TunableConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.util.FieldPoseUtils;
// import frc.robot.util.RunMode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final PIDController topPID;
  private final PIDController bottomPID;

  private final PIDController articulationPID;
  private final ArmFeedforward articulationFF;

  private final SimpleMotorFeedforward topFF;
  private final SimpleMotorFeedforward bottomFF;
  private final SysIdRoutine flywheelSysId;
  private final SysIdRoutine articulationSysId;

  private double topSetpointRPM = 0.0;
  private double bottomSetpointRPM = 0.0;
  private Rotation2d articulationSetpoint = Rotation2d.fromRadians(Math.PI);

  private SysIdRoutineLog.State sysIdState = SysIdRoutineLog.State.kNone;

  public Shooter(ShooterIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL:
      case REPLAY:
        topFF = new SimpleMotorFeedforward(KsTopFlywheel, KvTopFlywheel, KaTopFlywheel);
        bottomFF = new SimpleMotorFeedforward(KsBottomFlywheel, KvBottomFlywheel, KaBottomFlywheel);
        topPID = new PIDController(KpTopFlywheel, 0, KdTopFlywheel);
        bottomPID = new PIDController(KpBottomFlywheel, 0, KdBottomFlywheel);
        articulationPID = new PIDController(KpShooterArticulation, 0, KdShooterArticulation);
        articulationFF =
            new ArmFeedforward(
                KsShooterArticulation,
                KgShooterArticulation,
                KvShooterArticulation,
                KaIntakeArticulation);
        break;
      default:
        // simulated
        topFF = new SimpleMotorFeedforward(0, .003);
        bottomFF = new SimpleMotorFeedforward(0, .003);
        topPID = new PIDController(0.7, 0, 0);
        bottomPID = new PIDController(0.7, 0, 0);
        articulationPID = new PIDController(0.1, 0, 0);
        articulationFF = new ArmFeedforward(0, 0, 0, 0);
        break;
    }

    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> sysIdState = state),
            new SysIdRoutine.Mechanism(
                voltageMeasure -> {
                  io.setTopVoltage(voltageMeasure.in(Volts));
                  io.setBottomVoltage(voltageMeasure.in(Volts));
                },
                null,
                this));

    articulationSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> sysIdState = state),
            new SysIdRoutine.Mechanism(
                voltageMeasure -> {
                  io.setArticulationVoltage(voltageMeasure.in(Volts));
                },
                null,
                this));

    setArticulation(Rotation2d.fromDegrees(90 + 20));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/TopSetpointRPM", topSetpointRPM);
    Logger.recordOutput(
        "Shooter/TopRPM", Units.radiansPerSecondToRotationsPerMinute(inputs.topVelocityRadPerSec));

    Logger.recordOutput("Shooter/BottomSetpointRPM", bottomSetpointRPM);
    Logger.recordOutput(
        "Shooter/BottomRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.bottomVelocityRadPerSec));

    Logger.recordOutput("Shooter/ArticulationSetpoint", articulationSetpoint);
    Logger.recordOutput("Shooter/Articulation", inputs.articulationPosition);

    Logger.recordOutput("Shooter/RealMechanism", getMechanism(inputs.articulationPosition));
    Logger.recordOutput("Shooter/TargetMechanism", getMechanism(articulationSetpoint));

    Logger.recordOutput("Shooter/SysIdState", sysIdState.toString());

    if (sysIdState != SysIdRoutineLog.State.kNone) return;

    io.setTopVoltage(
        topPID.calculate(
                inputs.topVelocityRadPerSec, rotationsPerMinuteToRadiansPerSecond(topSetpointRPM))
            + topFF.calculate(rotationsPerMinuteToRadiansPerSecond(topSetpointRPM)));

    io.setBottomVoltage(
        bottomPID.calculate(
                inputs.bottomVelocityRadPerSec,
                rotationsPerMinuteToRadiansPerSecond(bottomSetpointRPM))
            + bottomFF.calculate(rotationsPerMinuteToRadiansPerSecond(bottomSetpointRPM)));

    io.setArticulationVoltage(
        articulationFF.calculate(articulationSetpoint.getRadians(), 0)
            + articulationPID.calculate(
                inputs.articulationPosition.getRadians(), articulationSetpoint.getRadians()));
  }

  private Mechanism2d getMechanism(Rotation2d articulation) {
    Mechanism2d mechanism = new Mechanism2d(DriveConstants.WidthWithBumpersX, 1);
    MechanismRoot2d mechRoot =
        mechanism.getRoot(
            "shooter",
            DriveConstants.WidthWithBumpersX / 2 + ShooterConstants.PivotX,
            ShooterConstants.PivotHeight);
    MechanismLigament2d elevator =
        mechRoot.append(new MechanismLigament2d("shooter pivot", 0.3, articulation.getDegrees()));
    return mechanism;
  }

  public void setFlywheelSetpointRPM(double top, double bottom) {
    topSetpointRPM = top;
    bottomSetpointRPM = bottom;
  }

  public void setArticulation(Rotation2d articulation) {
    articulationSetpoint = Rotation2d.fromRadians(angleModulus(articulation.getRadians()));
  }

  /** does not work if setpoint is zero!! */
  @AutoLogOutput
  public boolean flywheelIsAtSetpoint() {
    var topError =
        Math.abs(
            Units.radiansPerSecondToRotationsPerMinute(inputs.topVelocityRadPerSec)
                - topSetpointRPM);
    var bottomError =
        Math.abs(
            Units.radiansPerSecondToRotationsPerMinute(inputs.bottomVelocityRadPerSec)
                - bottomSetpointRPM);

    return (bottomError < VelocityToleranceRPM) && (topError < VelocityToleranceRPM);
  }

  @AutoLogOutput
  public boolean articulationIsAtSetpoint() {
    return Math.abs(inputs.articulationPosition.minus(articulationSetpoint).getRadians())
        < ArticulationToleranceRad;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheelSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheelSysId.dynamic(direction);
  }

  /** set articulation to point towards source + run flywheel */
  public Command prepareToShoot(Supplier<Pose2d> poseSupplier) {
    return new InstantCommand(
            () -> setFlywheelSetpointRPM(-ShootingVelocityRPM, ShootingVelocityRPM), this)
        .andThen(
            new RunCommand(
                () ->
                    setArticulation(
                        Rotation2d.fromRadians(
                            ArticulationMap.get(
                                poseSupplier
                                    .get()
                                    .getTranslation()
                                    .getDistance(
                                        FieldPoseUtils.flipTranslationIfRed(
                                            FieldConstants.SpeakerCloseSideCenter))))),
                this))
        .until(() -> flywheelIsAtSetpoint() && articulationIsAtSetpoint());
  }

  /** positive is outwards */
  public Command runAtRPM(double RPM) {
    return runAtRPM(() -> RPM);
  }

  public Command runAtRPM(double top, double bottom) {
    return runAtRPM(() -> top, () -> bottom);
  }

  public Command setRPM(double RPM) {
    return new InstantCommand(() -> setFlywheelSetpointRPM(-RPM, RPM));
  }

  /** positive is outwards */
  public Command runAtRPM(Supplier<Double> RPMSupplier) {
    return runAtRPM(() -> -RPMSupplier.get(), RPMSupplier);
  }

  public Command runAtRPM(Supplier<Double> topSupplier, Supplier<Double> bottomSupplier) {
    return new RunCommand(
            () -> {
              setFlywheelSetpointRPM(topSupplier.get(), bottomSupplier.get());
            },
            this)
        .until(this::flywheelIsAtSetpoint);
  }
}