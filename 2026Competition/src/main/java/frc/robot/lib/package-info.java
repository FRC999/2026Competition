/**
 * Hardware-independent geometry, shot planning, input, readiness and bounded-state policies.
 * Distance is meters and time is seconds unless a name states otherwise. Hood setpoints are radians;
 * turret setpoints are degrees; shooter setpoints are motor RPM. A calculated shot is not permission
 * to feed. Stateful helpers are confined to the robot scheduler thread unless explicitly documented.
 */
package frc.robot.lib;
