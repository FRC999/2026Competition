package frc.robot.OdometryUpdates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryConstants {

  /** Alliance-specific initial yaws (field-centric) for seeding MegaTag2. */
  public static final Rotation2d INITIAL_YAW_RED  = Rotation2d.fromDegrees(  180);
  public static final Rotation2d INITIAL_YAW_BLUE = Rotation2d.fromDegrees(0);
  public static final Rotation2d TELEOP_YAW_RED = Rotation2d.fromDegrees(180);
  public static final Rotation2d TELEOP_YAW_BLUE = Rotation2d.fromDegrees(0);

  /** Reject obviously invalid field poses before vision fusion. */
  public static final double MAX_REASONABLE_FIELD_COORD_ABS_METERS = 100.0;
  /** Hold Quest-primary mode briefly through short dropouts before falling back to LL. */
  public static final double QUEST_LOSS_HOLD_SEC = 5.0;
  /** Maximum age of the last unread Quest frame before Quest is treated as stale. */
  public static final double QUEST_STALE_TIMEOUT_SEC = 0.5;
  /** Default duration that all LLs must be blind before a reanchor is armed. */
  public static final double TAG_LOSS_REANCHOR_ARM_DELAY_SEC_DEFAULT = 2.0;
  /** Dashboard key for tuning the post-tag-loss reanchor arm delay. */
  public static final String TAG_LOSS_REANCHOR_ARM_DELAY_DASHBOARD_KEY =
      "Odometry/TagLossReanchorArmDelaySec";

  public static Rotation2d initialYawForAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red ? INITIAL_YAW_RED : INITIAL_YAW_BLUE;
  }

  public static Rotation2d teleopYawForAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red ? TELEOP_YAW_RED : TELEOP_YAW_BLUE;
  }

}
