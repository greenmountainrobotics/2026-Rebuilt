package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class AprilTagConstants {
  private static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final int[] SCORING_TAG_IDS = {};
  private static final int[] PICKUP_TAG_IDS = {}; // low kirk could get away w/ out ts
}
