package frc.robot.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Synthetic rotary load: no gravity, friction, fuel or measured inertia is implied. */
public final class RotaryMotorSim {
  private final DCMotorSim model;
  private final double motorRotationsPerMechanismRotation;

  /**
   * Creates a Kraken X60 load; construction on real hardware is rejected.
   * @param motorCount mechanically coupled motors contributing torque/load
   * @param mechanismInertiaKgM2 mechanism-side inertia, an unvalidated model assumption
   * @param reduction motor rotations per mechanism rotation (e.g. 11 for an 11:1 reduction)
   */
  public RotaryMotorSim(int motorCount, double mechanismInertiaKgM2, double reduction) {
    if (!RobotBase.isSimulation()) throw new IllegalStateException("Simulation model on real robot");
    motorRotationsPerMechanismRotation = reduction;
    var motors = DCMotor.getKrakenX60(motorCount);
    // WPILib takes inertia BEFORE reduction. Its outputs are mechanism, not rotor, units.
    model = new DCMotorSim(LinearSystemId.createDCMotorSystem(motors, mechanismInertiaKgM2, reduction), motors);
  }

  /** Advances from actual simulated motor voltage using a finite step in (0, 0.05] seconds. */
  public void update(double volts, double seconds) {
    if (!Double.isFinite(volts) || !Double.isFinite(seconds) || seconds <= 0 || seconds > .05)
      throw new IllegalArgumentException("Invalid simulation input");
    model.setInputVoltage(volts);
    model.update(seconds);
  }

  /** Raw motor rotor turns for CTRE SimState; do not apply the reduction again at the caller. */
  public double rotorPositionRotations() {
    return model.getAngularPositionRotations() * motorRotationsPerMechanismRotation;
  }

  /** Raw motor rotor rotations per second for CTRE SimState. */
  public double rotorVelocityRps() {
    return model.getAngularVelocityRadPerSec() / (2 * Math.PI) * motorRotationsPerMechanismRotation;
  }

  /** Battery model omits regeneration; reverse rotation must still draw positive current. */
  public double getCurrentDrawAmps() { return Math.max(0, model.getCurrentDrawAmps()); }
}
