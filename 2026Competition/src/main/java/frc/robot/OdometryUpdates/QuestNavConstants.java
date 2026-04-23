package frc.robot.OdometryUpdates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

public final class QuestNavConstants {
  private QuestNavConstants() {}

  public static final Transform2d ROBOT_TO_QUEST =
      new Transform2d(-0.225, 0.22, Rotation2d.fromDegrees(135.0));
  public static final Transform3d ROBOT_TO_QUEST_3D =
      new Transform3d(-0.225, 0.22, 0.3175, new Rotation3d(0.0, 0.0, Math.toRadians(135.0)));

  public static final Pose2d NULL_POSE = new Pose2d(-1000.0, -1000.0, Rotation2d.kZero);
  public static final Pose3d NULL_POSE_3D = new Pose3d(-1000.0, -1000.0, -1000.0, Rotation3d.kZero);
  public static final Pose3d ROBOT_ZERO_POSE_3D = new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero);

  public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02,
          0.02,
          0.035);
}
