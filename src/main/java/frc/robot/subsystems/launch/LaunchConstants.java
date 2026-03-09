// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launch;

public class LaunchConstants {
  /** CAN IDs for the two Kraken (Talon FX) launch motors. */
  public static final int launchLeaderCanId = 7;

  public static final int launchFollowerCanId = 8;

  /** Supply current limit (amps) per motor. */
  public static final int currentLimit = 60;

  /** Whether the follower is inverted relative to the leader. */
  public static final boolean followerInverted = false;

  /** Default shoot velocity in rad/s (tune for your mechanism). */
  public static final double shootVelocityRadPerSec = 500.0;

  /** Feedforward: volts for static friction and per rad/s. */
  public static final double kS = 0.05;

  public static final double kV = 0.012;
}
