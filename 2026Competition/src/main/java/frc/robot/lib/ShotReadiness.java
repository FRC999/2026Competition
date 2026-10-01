package frc.robot.lib;

/** Continuous feed permission, checked on every loop (including while already firing). */
public final class ShotReadiness {
  private ShotReadiness() {}

  public static boolean canFeed(boolean requested, boolean validSolution, boolean turretAimed,
      boolean turretPositionTrusted, boolean shooterReady, boolean hoodReady,
      boolean motionAllowed, boolean suppressed, boolean coolingDown) {
    return requested && validSolution && turretAimed && turretPositionTrusted && shooterReady
        && hoodReady && motionAllowed && !suppressed && !coolingDown;
  }
}
