package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMotorSubsystem extends SubsystemBase {
  public static final int kCanId = 55;

  private final boolean isSim = RobotBase.isSimulation();

  // Rotor inertia reflected to motor shaft (tune as needed for "feel").
  private static final double kRotorInertia = 0.001;

  // Simple supply drop model (ohms). Optional, but harmless.
  private static final double kMotorResistance = 0.002;

  private final TalonFX m_motor = new TalonFX(kCanId);
  private final DutyCycleOut m_dutyReq = new DutyCycleOut(0.0);

  // Simulation members (only used in sim)
  private final TalonFXSimState m_simState;
  private final DCMotorSim m_motorSim;

  public KrakenMotorSubsystem() {
    m_motor.getConfigurator().apply(new TalonFXConfiguration());

    if (RobotBase.isSimulation()) {
      m_simState = m_motor.getSimState();

      var gearbox = DCMotor.getKrakenX60Foc(1);
      m_motorSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, kRotorInertia, 1.0),
          gearbox
      );
    } else {
      m_simState = null;
      m_motorSim = null;
    }
  }

  public void setDutyCycle(double dutyCycle) {
    m_motor.setControl(m_dutyReq.withOutput(dutyCycle));
  }

  public void stop() {
    setDutyCycle(0.0);
  }

  public double getSimCurrentDrawAmps() {
    if (!isSim) {
      return 0.0;
    }
    return m_motor.getSimState().getSupplyCurrent();
  }


  @Override
  public void simulationPeriodic() {
    // WPILib calls this automatically in simulation for each Subsystem. No Robot.java edits needed.
    if (m_simState == null || m_motorSim == null) return;

    // Feed CTRE’s motor voltage output into WPILib’s motor physics model
    m_motorSim.setInputVoltage(m_simState.getMotorVoltage());
    m_motorSim.update(0.02); // Scheduler period (typical 20ms). You can compute dt if you prefer.

    // Update simulated sensor state for Phoenix
    final double posRot = m_motorSim.getAngularPositionRotations();
    final double velRps = Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec());

    m_simState.setRawRotorPosition(posRot);
    m_simState.setRotorVelocity(velRps);

    // Optional: approximate battery sag
    m_simState.setSupplyVoltage(RoboRioSim.getVInVoltage());
  }
}
