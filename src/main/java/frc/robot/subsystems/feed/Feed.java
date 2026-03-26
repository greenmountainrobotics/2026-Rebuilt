// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feed extends SubsystemBase {
  private final FeedIO io;
  private final FeedIOInputsAutoLogged inputs = new FeedIOInputsAutoLogged();

  public Feed(FeedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feed", inputs);
  }

  /** Runs the feed motor open-loop at the given voltage (e.g. -12 to 12). */
  public void runVoltage(double volts) {
    io.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  /** Stops the feed (0 V). */
  public void stop() {
    runVoltage(0.0);
  }

  /** Returns a command that runs the feed at the given voltage until interrupted. */
  public Command runAtVoltage(double volts) {
    return run(() -> runVoltage(volts)).finallyDo(() -> stop());
  }
}
