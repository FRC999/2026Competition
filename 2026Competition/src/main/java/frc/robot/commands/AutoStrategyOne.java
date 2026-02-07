// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.DeferredCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AutoStrategyOne extends SequentialCommandGroup {
//   /** Creates a new AutoStrategyOne. */
//   public AutoStrategyOne() {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new DeferredCommand(
//             () -> RobotContainer.runTrajectory2PosesSlow(
//                 RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(7.219, 6.130, Rotation2d.k180deg)), // if vision is not available at the start, use that pose
//                 RobotPoseConstants.visionRobotPoses.get("RobotBluReef3Right"),
//                 false),
//                 Set.of)
//             .alongWith(new AutoShootUntilEmpty())
//     );
//   }
// }
