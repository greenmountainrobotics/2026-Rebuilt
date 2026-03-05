package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static int TopMotorCanID = 0;
  public static int BottomMotorCanID = 0;

  public static double PivotHeight = Units.inchesToMeters(10.197);
  public static double PivotX = Units.inchesToMeters(0);

  public static final double VelocityToleranceRPM = 150;
  public static final double ArticulationToleranceRad = 0.02;

  public static final double ShootingVelocityRPM = 6000;
  public static final double AmpVelocityRPM = 900;
  public static final double AmpTopToBottomRatio = 4.5;
  public static final double ShooterGearRatio = 1.0 / 1.25;

  // measured from bottom edge of shooter facing forwards
  public static final Rotation2d AbsoluteEncoderOffset =
      Rotation2d.fromRadians(2.67).unaryMinus().plus(Rotation2d.fromRadians(Math.PI));

  /** k: m, v: rad */
  // UNUSED
  public static final InterpolatingDoubleTreeMap ArticulationMap = new InterpolatingDoubleTreeMap();

  static {
    ArticulationMap.put(1.0, Math.PI / 2);
    ArticulationMap.put(2.0, Math.PI / 4);
    ArticulationMap.put(5.0, Math.PI);
  }
}