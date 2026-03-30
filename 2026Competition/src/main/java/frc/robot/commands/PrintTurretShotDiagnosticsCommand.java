package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.TurretHelpers;

public class PrintTurretShotDiagnosticsCommand extends InstantCommand {

  public PrintTurretShotDiagnosticsCommand() {
    // No subsystem requirements needed; this is print-only.
  }

  @Override
  public void initialize() {
    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    Rotation2d robotHeading = robotPose.getRotation();

    Translation2d turretCenterField =
        robotPose.getTranslation().plus(
            Constants.OperatorConstants.TurretGeometry
                .TURRET_PIVOT_OFFSET_FROM_ROBOT_ORIGIN_METERS
                .rotateBy(robotHeading));

    boolean isRed =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Constants.FieldTargets.AimTarget aimTarget =
        RobotContainer.autoShootSupervisorSubsystem.getCurrentAimTarget();

    Translation2d hubCenterField = new Translation2d(
        aimTarget.getX(isRed),
        aimTarget.getY(isRed));

    double turretAngleRelativeToZeroDeg =
        RobotContainer.turretSubsystem.getAngleDeg();

    double absoluteTurretFieldDeg = MathUtil.inputModulus(
        robotHeading.getDegrees()
            + Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG
            + turretAngleRelativeToZeroDeg,
        -180.0,
        180.0);

    double distanceMeters = turretCenterField.getDistance(hubCenterField);

    TurretHelpers.Solution solution =
        RobotContainer.autoShootSupervisorSubsystem.calculateDiagnosticSolution();

    double turretRelativeAngleDeg = RobotContainer.turretSubsystem.getAngleDeg();

     double expectedTurretRelativeAngleDeg =
    MathUtil.inputModulus(
        absoluteTurretFieldDeg
            - robotHeading.getDegrees()
            - Constants.OperatorConstants.Turret.ZERO_OFFSET_FROM_ROBOT_FWD_DEG,
        -180.0,
        180.0);

    System.out.println("========================================");
    System.out.println("TURRET / SHOT DIAGNOSTICS");
    System.out.println("========================================");

    System.out.printf(
        "1. Turret center field position: x=%.4f m, y=%.4f m%n",
        turretCenterField.getX(),
        turretCenterField.getY());

    System.out.printf(
        "2. Absolute turret angle from field POV: %.3f deg%n",
        absoluteTurretFieldDeg);

    System.out.printf(
        "3. Distance from turret center to hub center: %.4f m%n",
        distanceMeters);

    if (solution == null || !solution.valid
        || !Double.isFinite(solution.shooterRpmCommand)
        || !Double.isFinite(solution.hoodCommandAngleRad)) {

      System.out.println(
          "4. No valid distance-table solution is currently available.");

    } else {
      double hoodDeg = Math.toDegrees(solution.hoodCommandAngleRad);

      System.out.printf(
          "4. Apply hood=%.3f deg, shooter=%.1f RPM%n",
          hoodDeg,
          solution.shooterRpmCommand);
    
    }

    // System.out.printf(
    // "5. Turret relative angle from turret zero POV: %.3f deg%n",
    // turretRelativeAngleDeg);

    // System.out.printf(
    // "6. Expected turret relative angle to hub: %.3f deg%n",
    // expectedTurretRelativeAngleDeg);

    // System.out.println("========================================");
  }
}