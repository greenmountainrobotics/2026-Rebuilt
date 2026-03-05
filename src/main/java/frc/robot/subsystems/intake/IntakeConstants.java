// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  /** CAN IDs for the two Kraken (Talon FX) intake motors. */
  public static final int intakeLeaderCanId = 5;

  public static final int intakeFollowerCanId = 6;

  /** Supply current limit (amps) per motor. */
  public static final int currentLimit = 40;

  /** Whether the follower is inverted relative to the leader (e.g. opposite side of roller). */
  public static final boolean followerInverted = false;
}
