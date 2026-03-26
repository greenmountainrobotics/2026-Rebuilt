// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Shooter IO implementation for three Kraken (Talon FX) motors with velocity control. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader = new TalonFX(shooterLeaderCanId);
  private final TalonFX follower = new TalonFX(shooterFollowerCanId);
  private final TalonFX follower2 = new TalonFX(shooterFollower2CanId);

  private final StatusSignal<Angle> position = leader.getPosition();
  private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Current> follower2Current = follower2.getSupplyCurrent();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    config.MotorOutput.Inverted =
        followerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));

    config.MotorOutput.Inverted =
        follower2Inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> follower2.getConfigurator().apply(config, 0.25));

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    follower2.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, leaderCurrent, followerCurrent, follower2Current);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
    follower2.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, leaderCurrent, followerCurrent, follower2Current);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {
          leaderCurrent.getValueAsDouble(),
          followerCurrent.getValueAsDouble(),
          follower2Current.getValueAsDouble(),
        };
  }

  @Override
  public void setVelocity(double radPerSec, double ffVolts) {
    leader.setControl(
        velocityRequest.withVelocity(Units.radiansToRotations(radPerSec)).withFeedForward(ffVolts));
  }
}
