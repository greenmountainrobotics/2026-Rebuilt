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
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {}

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command tankDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.tankDriveIK(x, z, true);
          // Apply output
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }

  /**
   * Runs the drive motors at a constant low speed for a set duration (e.g. burn-in). Stops when the
   * timeout is reached or the command is cancelled.
   */
  public static Command runMotorsForDuration(
      Drive drive, double speedMetersPerSec, double timeoutSeconds) {
    return Commands.run(() -> drive.runClosedLoop(speedMetersPerSec, speedMetersPerSec), drive)
        .withTimeout(timeoutSeconds)
        .finallyDo(interrupted -> drive.stop());
  }

  /** Drive to a target pose using the custom pose PID loop in {@link Drive}. */
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

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runOpenLoop(voltage, voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
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
