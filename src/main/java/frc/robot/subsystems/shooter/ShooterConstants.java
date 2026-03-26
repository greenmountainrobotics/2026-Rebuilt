// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

public class ShooterConstants {
  /** CAN IDs for the three Kraken (Talon FX) shooter motors. */
  public static final int shooterLeaderCanId = 7;

  public static final int shooterFollowerCanId = 8;
  public static final int shooterFollower2CanId = 9;

  /** Supply current limit (amps) per motor. */
  public static final int currentLimit = 60;

  /** Whether the first follower is inverted relative to the leader. */
  public static final boolean followerInverted = false;
  /** Whether the second follower is inverted relative to the leader. */
  public static final boolean follower2Inverted = false;

  /** Default shoot velocity in rad/s (tune for your mechanism). */
  public static final double shootVelocityRadPerSec = 500.0;

  /** Feedforward: volts for static friction and per rad/s. */
  public static final double kS = 0.05;

  public static final double kV = 0.012;
}
