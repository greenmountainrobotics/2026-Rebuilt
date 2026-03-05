package frc.robot.subsystems.launch;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public Rotation2d articulationPosition = new Rotation2d();
    public double articulationAppliedVolts = 0.0;
    public double[] articulationCurrentAmps = new double[] {};

    public double bottomPositionRad = 0.0;
    public double bottomVelocityRadPerSec = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomCurrentAmps = new double[] {};

    public double topPositionRad = 0.0;
    public double topVelocityRadPerSec = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topCurrentAmps = new double[] {};
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setArticulationVoltage(double volts) {}

  public default void setTopVoltage(double volts) {}

  public default void setBottomVoltage(double volts) {}
}