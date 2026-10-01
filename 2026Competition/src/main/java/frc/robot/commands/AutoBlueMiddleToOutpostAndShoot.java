package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.ShotPlanner.Mode;

/** Finish the route, then shoot for the remaining autonomous budget. */
public class AutoBlueMiddleToOutpostAndShoot extends SequentialCommandGroup {
  public AutoBlueMiddleToOutpostAndShoot() {
    addCommands(
        RobotContainer.followCompetitionPath("BlueMiddle_BlueOutpost", false, PrecisionPathCommands.FieldFrame.ALLIANCE).deadlineFor(new WaitCommand(1).andThen(new RepeatIntakeRetractDeploy())),
        new ShootWhileHeld(Mode.MOVING_AUTO, false));
  }
}
