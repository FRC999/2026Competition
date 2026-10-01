/**
 * Logged PhotonVision IO, observation validation and disabled absolute-pose initialization.
 * Camera observations use blue-origin field coordinates and FPGA capture seconds. Raw field-to-camera
 * solves and field-to-robot poses are distinct. Fusion requires measured camera extrinsics and matching
 * field-layout acknowledgment. Enabled heading remains gyro-owned; a camera connection is not pose trust.
 */
package frc.robot.subsystems.vision;
