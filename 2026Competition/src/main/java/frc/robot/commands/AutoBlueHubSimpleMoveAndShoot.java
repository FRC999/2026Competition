// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueHubSimpleMoveAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoBlueHubSimpleMoveAndShoot. */
  public AutoBlueHubSimpleMoveAndShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueHubMiddle_BlueAllianceMiddle", false, false),
      new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
        .raceWith(new WaitCommand(4)),
      new RetractIntakeSequence(),
      new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
        .raceWith(new WaitCommand(7))
    );
  }
}
