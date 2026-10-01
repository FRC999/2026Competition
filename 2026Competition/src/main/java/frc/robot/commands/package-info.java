/**
 * Scheduler-owned robot actions. Declare every controlled subsystem as a requirement and clean up
 * outputs/temporary modes in end(boolean), including timeout and interruption. A group owns its full
 * requirement union until it ends. Competition route stops and corrective precision alignment are
 * distinct policies; neither treats timeout as arrival. Operator bindings are enabled-teleop only.
 */
package frc.robot.commands;
