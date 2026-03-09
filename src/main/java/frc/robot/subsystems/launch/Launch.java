// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.launch;

import static frc.robot.subsystems.launch.LaunchConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launch extends SubsystemBase {
  private final LaunchIO io;
  private final LaunchIOInputsAutoLogged inputs = new LaunchIOInputsAutoLogged();

  public Launch(LaunchIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launch", inputs);
  }

  /** Runs both launch motors at the given velocity (rad/s) with feedforward. */
  public void setVelocity(double radPerSec) {
    double ffVolts = kS * Math.signum(radPerSec) + kV * radPerSec;
    io.setVelocity(radPerSec, ffVolts);
  }

  /** Stops the launcher (0 velocity). */
  public void stop() {
    setVelocity(0.0);
  }

  /** Returns a command that runs the launcher at the given velocity until interrupted. */
  public Command runAtVelocity(double radPerSec) {
    return run(() -> setVelocity(radPerSec)).finallyDo(() -> stop());
  }
}
