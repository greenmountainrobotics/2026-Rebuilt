// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feed;

public class FeedConstants {
  /** CAN IDs for the two Kraken (Talon FX) feed motors. */
  public static final int feedLeaderCanId = 10;

  public static final int feedFollowerCanId = 11;

  /** Supply current limit (amps) per motor. */
  public static final int currentLimit = 40;

  /** Whether the follower is inverted relative to the leader. */
  public static final boolean followerInverted = false;
}
