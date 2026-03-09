// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launch;

import org.littletonrobotics.junction.AutoLog;

public interface LaunchIO {
  @AutoLog
  public static class LaunchIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LaunchIOInputs inputs) {}

  /** Runs both launch motors at the given velocity (rad/s) with feedforward volts. */
  public default void setVelocity(double radPerSec, double ffVolts) {}
}
