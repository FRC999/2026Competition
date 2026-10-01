package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants.Turret;
import frc.robot.RobotContainer;
import frc.robot.lib.AimGeometry;

/** Read-only diagnostics use exactly the same shot planner as control. */
public class PrintTurretShotDiagnosticsCommand extends InstantCommand {
  @Override public void initialize() {
    var pose = RobotContainer.driveSubsystem.getPose();
    var pivot = AimGeometry.pivot(pose);
    var solution = RobotContainer.autoShootSupervisorSubsystem.calculateDiagnosticSolution();
    double actualFieldDegrees = MathUtil.inputModulus(pose.getRotation().getDegrees()
        + Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG + RobotContainer.turretSubsystem.getAngleDeg(), -180, 180);
    System.out.printf("Shot plan: pivot=(%.4f, %.4f) m; actual field bearing=%.3f deg; valid=%s; model=%s%n",
        pivot.getX(), pivot.getY(), actualFieldDegrees, solution.valid(), solution.leadSource());
    if (solution.valid()) System.out.printf(
        "Requested turret=%.3f deg; lookup distance=%.4f m; hood=%.3f deg; shooter=%.1f RPM%n",
        solution.turretDegrees(), solution.distanceMeters(), Math.toDegrees(solution.hoodCommandAngleRad()),
        solution.shooterRpmCommand());
  }
}
