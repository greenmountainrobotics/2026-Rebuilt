// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 5;
  public static final double trackWidth = Units.inchesToMeters(26.0);

  public static final int pigeonCanId = 11;
  public static final int leftLeaderCanId = 4;
  public static final int leftFollowerCanId = 3;
  public static final int rightLeaderCanId = 2;
  public static final int rightFollowerCanId = 1;

  // Motor configuration
  public static final int currentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(6.0);
  public static final double motorReduction = 8.46;
  public static final boolean leftInverted = false;
  public static final boolean rightInverted = true;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public static final double realKp = 0.2;
  public static final double realKd = 0.001; // keep small dumb dumb silly silly clay
  public static final double realKs = 0.5;
  public static final double realKv = 0;

  public static final double simKp = 0.0;
  public static final double simKd = 0.0;
  public static final double simKs = 0.3;
  public static final double simKv = 2.5;

  public static final double robotMassKg = 103;
  public static final double robotMOI = 6;
  public static final double wheelCOF = .9;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              gearbox.withReduction(motorReduction),
              currentLimit,
              2),
          trackWidth);

  // auto alignment stuffs
  public static final double scoringOffsetX = 0; // distance from tag fuh
  public static final double scoringOffsetY = 0;

  public static final double pickupOffsetX = 0;
  public static final double pickupOffsetY = 0;

  // Toleruh
  public static final double driveTolerance = 0.05;
  public static final double thetaToleranceRad = 0.03;
}
