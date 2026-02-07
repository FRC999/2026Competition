// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper.AutoDesiredPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyFour extends SequentialCommandGroup {
  /** Creates a new AutoStrategyFour. */
  public AutoStrategyFour() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        RobotContainer.runTrajectory2Poses(RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
            AutoDesiredPoses.BlueNeutralLeft, false),
        new StartIntake(),
        RobotContainer.runTrajectory2Poses(
            RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
            new Pose2d(AutoDesiredPoses.BlueNeurtralMiddle.getTranslation(), new Rotation2d(Math.toRadians(-90))),
            false),
        RobotContainer.runTrajectory2Poses(
            RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
            AutoDesiredPoses.BlueNeutralRight, 
            false),
        new AutoShootUntilEmpty() // TODO: NEED TO ADD CLIMBING COMMANDS HERE
    );
  }
}
