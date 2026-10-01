/**
 * Hardware ownership and per-loop control. CTRE configuration, sensor trust and guarded output belong
 * here; commands express operator/auto intent. Drive owns field-reference trust and FPGA-to-CTRE time
 * conversion. AutoShootSupervisor coordinates continuous readiness without competing with an external
 * command owner. Simulation callbacks use isolated synthetic models and must remain real-robot guarded.
 */
package frc.robot.subsystems;
