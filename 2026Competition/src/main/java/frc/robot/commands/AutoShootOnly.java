package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.ShotPlanner.Mode;

/** Shoot from the current referenced pose while holding the chassis stopped. */
public class AutoShootOnly extends SequentialCommandGroup {
  public AutoShootOnly() {
    addCommands(new ShootWhileHeld(Mode.MOVING_AUTO, false)
        .alongWith(Commands.run(RobotContainer.driveSubsystem::stop, RobotContainer.driveSubsystem)));
  }
}
