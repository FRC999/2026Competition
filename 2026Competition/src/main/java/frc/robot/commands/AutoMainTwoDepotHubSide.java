// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMainTwoDepotHubSide extends SequentialCommandGroup {
  /** Creates a new AutoMainTwoDepotHubSide. */
  public AutoMainTwoDepotHubSide() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              TrajectoryHelper.AutoDesiredPoses.BlueBumpLeft,
              //new Pose2d(3.884, 6.966, new Rotation2d()),
              //RobotContainer.driveSubsystem.getPose(),
              TrajectoryHelper.AutoDesiredPoses.BlueDepot),
          Set.of(RobotContainer.driveSubsystem))
            ,
            //.alongWith(new AutoShootUntilEmpty())
            //.alongWith(new WaitCommand(1))
            //.andThen(new StartIntake()),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueDepot_BlueTrenchLeft", false, false)
            //.andThen(new StopIntake())
            //.alongWith(new AutoShootUntilEmpty()),
            ,
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchLeft_BlueTrenchLeft2", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchLeft2_BlueNeutralHubLeft", false, false),
          //new StartIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralHubLeft_BlueTower", false, false)
          //new StopIntake(),
            //.alongWith(new AutoShootUntilEmpty())

    );
  }
}
