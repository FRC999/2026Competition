// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper.AutoDesiredPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyFive extends SequentialCommandGroup {
  /** Creates a new AutoStrategyFive. */
  public AutoStrategyFive() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new InstantCommand(()->RobotContainer.driveSubsystem.resetPose(new Pose2d(3.522, 6.45, new Rotation2d(0)))),
       RobotContainer.runTrajectory2Poses(
            true,
            RobotContainer.driveSubsystem.getPose(),
            new Pose2d(AutoDesiredPoses.BlueBumpLeft2.getTranslation(), new Rotation2d(Math.toRadians(90)))),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpLeft2_BlueNeutralRight",false, false),
        //new StartIntake(),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRight_BlueNeutralLeft",false, false),
        //new StopIntake(),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralLeft_BlueBumpLeft",false, false),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpLeft_BlueDepot",false, false)
          //.alongWith(new AutoShootUntilEmpty()),
        ,RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueDepot_BlueTower",false, false)
        // TODO: NEED TO ADD CLIMBING COMMANDS HERE
    );
  }
}
