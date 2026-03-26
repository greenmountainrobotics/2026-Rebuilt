// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {}

  public static Command tankDrive(
      Drive drive, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    return Commands.run(
        () -> {
          double left = MathUtil.applyDeadband(leftSupplier.getAsDouble(), DEADBAND);
          double right = MathUtil.applyDeadband(rightSupplier.getAsDouble(), DEADBAND);
          drive.runClosedLoop(left * maxSpeedMetersPerSec, right * maxSpeedMetersPerSec);
        },
        drive);
  }

  /**
   * Arcade drive: forward and rotation from stick, full speed. Turbo (kV) is toggled via A in
   * RobotContainer. Optional brake supplier (0–1): left trigger scales commanded speed by (1 -
   * brake), so harder trigger = stronger brake.
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier) {
    return arcadeDrive(drive, forwardSupplier, rotationSupplier, () -> 0.0);
  }

  /**
   * Arcade drive with proportional brake. brakeSupplier should return 0–1 (e.g. left trigger);
   * commanded wheel speeds are scaled by (1 - brake), so harder trigger = harder brake.
   */
  public static Command arcadeDrive(
      Drive drive,
      DoubleSupplier forwardSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier brakeSupplier) {
    return Commands.run(
        () -> {
          double forward = MathUtil.applyDeadband(forwardSupplier.getAsDouble(), DEADBAND);
          double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), DEADBAND);
          double brake = MathUtil.clamp(brakeSupplier.getAsDouble(), 0.0, 1.0);
          var speeds = DifferentialDrive.arcadeDriveIK(forward, rotation, true);
          double scale = 1.0 - brake;
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec * scale,
              speeds.right * maxSpeedMetersPerSec * scale);
        },
        drive);
  }

  // ts was designed for just running the motors not auto stupid stupid clay
  public static Command runMotorsForDuration(
      Drive drive, double speedMetersPerSec, double timeoutSeconds) {
    return Commands.run(() -> drive.runClosedLoop(speedMetersPerSec, speedMetersPerSec), drive)
        .withTimeout(timeoutSeconds)
        .finallyDo(interrupted -> drive.stop());
  }

  public static Command driveToPose(
      Drive drive, Supplier<Pose2d> targetSupplier, boolean stopWhenDone) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.resetPoseController(targetSupplier.get()), drive),
        Commands.run(() -> drive.runPoseController(targetSupplier.get()), drive)
            .until(() -> drive.atPoseSetpoint(targetSupplier.get()))
            .finallyDo(
                interrupted -> {
                  if (stopWhenDone) {
                    drive.stop();
                  }
                }));
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // resuh data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),
        Commands.run( // acceleruh
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runOpenLoop(voltage, voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)
            // pruh resulh
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
