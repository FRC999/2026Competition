# Source change counts

Baseline: `6c4ecb4c196541236e7f3a702e2ad1099a094e1c` (Houston, confirmed by mentor).
Target: `5cedfe09beb8969dcadaf15bcfeb01459f0dcacc`.

Production Java only; physical source lines including comments/blanks. Added + deleted is churn, not unique edited lines. No rename detection; deleted legacy code is included.

| Attribution | Added | Deleted | Churn |
|---|---:|---:|---:|
| Vision migration | 1,713 | 4,192 | 5,905 |
| Simulation | 235 | 284 | 519 |
| Other fixes and strategy | 3,154 | 6,455 | 9,609 |
| Shared integration | 489 | 1,692 | 2,181 |
| TOTAL | 5,591 | 12,623 | 18,214 |

## Attribution convention

Every changed line appears once in the CSV. Simulation files, named simulation methods/fields and simulation imports take precedence. Vision includes the replacement localization stack, configuration, and retirement of Limelight/Quest helpers. Other includes precision driving, aim/shot algorithms, commands, mechanism fixes and retired dead code. Shared integration keeps Robot, RobotContainer, Constants, DriveSubsystem, ElasticHelpers and Telemetry changes unallocated except their explicit simulation scopes. Those files combine vision wiring with behavioral changes; calling all of them 'just PhotonVision' would be misleading. The vision and simulation buckets are conservative direct attributions; a unique causal allocation of every shared line is not possible from a final diff.

Tests, path/configuration data, build dependencies, calibration tools and documentation are excluded from production Java totals. Separate tracked-file counts follow. The generated report/CSV are excluded from those supplemental totals to avoid self-counting.

## Supplemental files

| Scope | Added | Deleted | Churn |
|---|---:|---:|---:|
| Documentation/skills/build/other | 2,537 | 475 | 3,012 |
| Java tests | 1,812 | 0 | 1,812 |
| Path/configuration data | 1,285 | 44 | 1,329 |
| Tools and Python tests | 579 | 0 | 579 |

## Production detail

| File | Attribution | Added | Deleted |
|---|---|---:|---:|
| `frc/robot/Constants.java` | Shared integration | 30 | 149 |
| `frc/robot/Controller.java` | Other fixes and strategy | 22 | 182 |
| `frc/robot/Main.java` | Other fixes and strategy | 3 | 0 |
| `frc/robot/OdometryUpdates/LLAprilTagConstants.java` | Vision migration | 0 | 68 |
| `frc/robot/OdometryUpdates/LLAprilTagSubsystem.java` | Vision migration | 0 | 497 |
| `frc/robot/OdometryUpdates/OdometryConstants.java` | Vision migration | 0 | 36 |
| `frc/robot/OdometryUpdates/OdometryUpdatesSubsystem.java` | Vision migration | 0 | 942 |
| `frc/robot/OdometryUpdates/QuestNavConstants.java` | Vision migration | 0 | 37 |
| `frc/robot/OdometryUpdates/QuestNavSubsystem.java` | Vision migration | 0 | 440 |
| `frc/robot/Robot.java` | Shared integration | 39 | 62 |
| `frc/robot/Robot.java` | Simulation | 5 | 1 |
| `frc/robot/RobotContainer.java` | Shared integration | 224 | 1034 |
| `frc/robot/Telemetry.java` | Shared integration | 0 | 126 |
| `frc/robot/commands/AutoBlueHubSimpleMoveAndShoot.java` | Other fixes and strategy | 7 | 4 |
| `frc/robot/commands/AutoBlueMiddleToOutpostAndShoot.java` | Other fixes and strategy | 5 | 39 |
| `frc/robot/commands/AutoBlueSimpleMoveAndShootLastResort.java` | Other fixes and strategy | 0 | 27 |
| `frc/robot/commands/AutoBlueTrenchToOutpostAndShoot.java` | Other fixes and strategy | 4 | 36 |
| `frc/robot/commands/AutoBlueWorlds.java` | Other fixes and strategy | 0 | 41 |
| `frc/robot/commands/AutoMainOneLeft.java` | Other fixes and strategy | 0 | 50 |
| `frc/robot/commands/AutoMainOneRight.java` | Other fixes and strategy | 0 | 87 |
| `frc/robot/commands/AutoMainOneRightBlue.java` | Other fixes and strategy | 17 | 19 |
| `frc/robot/commands/AutoMainOneRightRed.java` | Other fixes and strategy | 17 | 19 |
| `frc/robot/commands/AutoMainTwoDepotHubSide.java` | Other fixes and strategy | 0 | 51 |
| `frc/robot/commands/AutoMainTwoDepotMiddle.java` | Other fixes and strategy | 0 | 53 |
| `frc/robot/commands/AutoRedHubSimpleMoveAndShoot.java` | Other fixes and strategy | 0 | 29 |
| `frc/robot/commands/AutoRedSimpleMoveAndShootLastResort.java` | Other fixes and strategy | 0 | 26 |
| `frc/robot/commands/AutoRedTrenchToOutpostAndShoot.java` | Other fixes and strategy | 0 | 32 |
| `frc/robot/commands/AutoShootOnly.java` | Other fixes and strategy | 5 | 15 |
| `frc/robot/commands/AutoShootUntilEmpty.java` | Other fixes and strategy | 0 | 47 |
| `frc/robot/commands/AutoShootUntilEmptyExclusive.java` | Other fixes and strategy | 0 | 61 |
| `frc/robot/commands/AutoStrategyEight.java` | Other fixes and strategy | 0 | 38 |
| `frc/robot/commands/AutoStrategyFive.java` | Other fixes and strategy | 0 | 42 |
| `frc/robot/commands/AutoStrategyFour.java` | Other fixes and strategy | 0 | 43 |
| `frc/robot/commands/AutoStrategyOne.java` | Other fixes and strategy | 0 | 46 |
| `frc/robot/commands/AutoStrategySeven.java` | Other fixes and strategy | 0 | 45 |
| `frc/robot/commands/AutoStrategySix.java` | Other fixes and strategy | 0 | 39 |
| `frc/robot/commands/AutoStrategyThree.java` | Other fixes and strategy | 0 | 35 |
| `frc/robot/commands/AutoStrategyTwo.java` | Other fixes and strategy | 0 | 47 |
| `frc/robot/commands/AutoWorldsHubSweep.java` | Other fixes and strategy | 0 | 66 |
| `frc/robot/commands/AutoWorldsHubSweepBlue.java` | Other fixes and strategy | 19 | 21 |
| `frc/robot/commands/AutoWorldsHubSweepRed.java` | Other fixes and strategy | 17 | 19 |
| `frc/robot/commands/ClimbDown.java` | Other fixes and strategy | 0 | 37 |
| `frc/robot/commands/ClimbUp.java` | Other fixes and strategy | 0 | 37 |
| `frc/robot/commands/DeployAndRunIntakeWhileHeld.java` | Other fixes and strategy | 14 | 0 |
| `frc/robot/commands/DeployIntakeSequence.java` | Other fixes and strategy | 7 | 1 |
| `frc/robot/commands/DriveInterrupt.java` | Other fixes and strategy | 0 | 24 |
| `frc/robot/commands/DriveManuallyCommand.java` | Other fixes and strategy | 38 | 50 |
| `frc/robot/commands/DriveToPosePrecisionCommand.java` | Other fixes and strategy | 1010 | 0 |
| `frc/robot/commands/GuardedSysId.java` | Other fixes and strategy | 19 | 0 |
| `frc/robot/commands/InitialAutoDeployIntake.java` | Other fixes and strategy | 0 | 52 |
| `frc/robot/commands/InitialAutoDeployWhileHeld.java` | Other fixes and strategy | 8 | 2 |
| `frc/robot/commands/IntakePowerIn.java` | Other fixes and strategy | 0 | 52 |
| `frc/robot/commands/IntakePowerOut.java` | Other fixes and strategy | 0 | 49 |
| `frc/robot/commands/IntakeRezeroFromRetractedHardStop.java` | Other fixes and strategy | 23 | 59 |
| `frc/robot/commands/IntakeToPositionAndHold.java` | Other fixes and strategy | 0 | 56 |
| `frc/robot/commands/NoAuto_Auto.java` | Other fixes and strategy | 0 | 22 |
| `frc/robot/commands/PrecisionPathCommands.java` | Other fixes and strategy | 112 | 0 |
| `frc/robot/commands/PrintTurretShotDiagnosticsCommand.java` | Other fixes and strategy | 16 | 102 |
| `frc/robot/commands/PulseIntakeForBallSettle.java` | Other fixes and strategy | 0 | 73 |
| `frc/robot/commands/RepeatIntakeRetractDeploy.java` | Other fixes and strategy | 4 | 0 |
| `frc/robot/commands/RetractIntakeSequence.java` | Other fixes and strategy | 7 | 0 |
| `frc/robot/commands/RetractIntakeSequenceWithTimeout.java` | Other fixes and strategy | 0 | 48 |
| `frc/robot/commands/ReverseIntake.java` | Other fixes and strategy | 3 | 0 |
| `frc/robot/commands/ReverseShooterTemporary.java` | Other fixes and strategy | 15 | 64 |
| `frc/robot/commands/ReverseSpindexer.java` | Other fixes and strategy | 0 | 41 |
| `frc/robot/commands/ReverseTransfer.java` | Other fixes and strategy | 0 | 42 |
| `frc/robot/commands/ShootCalibrationBurstWhileHeld.java` | Other fixes and strategy | 0 | 136 |
| `frc/robot/commands/ShootWhileHeld.java` | Other fixes and strategy | 6 | 49 |
| `frc/robot/commands/ShooterAdjustRpmCommand.java` | Other fixes and strategy | 0 | 12 |
| `frc/robot/commands/ShooterEnableCommand.java` | Other fixes and strategy | 0 | 45 |
| `frc/robot/commands/StartIntake.java` | Other fixes and strategy | 3 | 1 |
| `frc/robot/commands/StopAtRouteEnd.java` | Other fixes and strategy | 70 | 0 |
| `frc/robot/commands/StopClimb.java` | Other fixes and strategy | 0 | 37 |
| `frc/robot/commands/StopIntake.java` | Other fixes and strategy | 3 | 0 |
| `frc/robot/commands/StopRobot.java` | Other fixes and strategy | 0 | 24 |
| `frc/robot/commands/TestAuto.java` | Other fixes and strategy | 0 | 28 |
| `frc/robot/commands/TestTurretAngleCommand.java` | Other fixes and strategy | 0 | 69 |
| `frc/robot/commands/TurretCalibrationJogCommand.java` | Other fixes and strategy | 0 | 30 |
| `frc/robot/commands/TurretGoToZeroCommand.java` | Other fixes and strategy | 5 | 1 |
| `frc/robot/commands/TurretToHub.java` | Other fixes and strategy | 0 | 33 |
| `frc/robot/commands/package-info.java` | Other fixes and strategy | 7 | 0 |
| `frc/robot/config/OffseasonVisionConfig.java` | Vision migration | 150 | 0 |
| `frc/robot/config/PrecisionConstants.java` | Other fixes and strategy | 37 | 0 |
| `frc/robot/config/VisionConstants.java` | Vision migration | 65 | 0 |
| `frc/robot/config/package-info.java` | Other fixes and strategy | 6 | 0 |
| `frc/robot/lib/AimGeometry.java` | Other fixes and strategy | 40 | 0 |
| `frc/robot/lib/DriverInput.java` | Other fixes and strategy | 19 | 0 |
| `frc/robot/lib/ElasticHelpers.java` | Shared integration | 26 | 166 |
| `frc/robot/lib/FieldRules.java` | Other fixes and strategy | 42 | 0 |
| `frc/robot/lib/FieldTargeting.java` | Other fixes and strategy | 83 | 0 |
| `frc/robot/lib/FreshPress.java` | Other fixes and strategy | 15 | 0 |
| `frc/robot/lib/HardStopHoming.java` | Other fixes and strategy | 26 | 0 |
| `frc/robot/lib/LimelightHelpers.java` | Vision migration | 0 | 1666 |
| `frc/robot/lib/MovingAimModel.java` | Other fixes and strategy | 66 | 0 |
| `frc/robot/lib/QuestHelpers.java` | Vision migration | 0 | 138 |
| `frc/robot/lib/ShooterReadinessPolicy.java` | Other fixes and strategy | 14 | 0 |
| `frc/robot/lib/ShotFlightTimeTable.java` | Other fixes and strategy | 43 | 0 |
| `frc/robot/lib/ShotIntent.java` | Other fixes and strategy | 25 | 0 |
| `frc/robot/lib/ShotPlanner.java` | Other fixes and strategy | 104 | 0 |
| `frc/robot/lib/ShotReadiness.java` | Other fixes and strategy | 33 | 0 |
| `frc/robot/lib/ShotTable.java` | Other fixes and strategy | 350 | 0 |
| `frc/robot/lib/TrajectoryHelper.java` | Other fixes and strategy | 0 | 65 |
| `frc/robot/lib/TurretHelpers.java` | Other fixes and strategy | 0 | 1414 |
| `frc/robot/lib/TurretMotionPolicy.java` | Other fixes and strategy | 27 | 0 |
| `frc/robot/lib/VisionHelpers.java` | Vision migration | 0 | 368 |
| `frc/robot/lib/package-info.java` | Other fixes and strategy | 7 | 0 |
| `frc/robot/package-info.java` | Other fixes and strategy | 7 | 0 |
| `frc/robot/simulation/RotaryMotorSim.java` | Simulation | 47 | 0 |
| `frc/robot/simulation/package-info.java` | Simulation | 7 | 0 |
| `frc/robot/subsystems/AutoShootSupervisorSubsystem.java` | Other fixes and strategy | 235 | 1446 |
| `frc/robot/subsystems/ClimbSubsystem.java` | Other fixes and strategy | 46 | 26 |
| `frc/robot/subsystems/ClimbSubsystem.java` | Simulation | 15 | 38 |
| `frc/robot/subsystems/DriveSubsystem.java` | Shared integration | 170 | 155 |
| `frc/robot/subsystems/DriveSubsystem.java` | Simulation | 22 | 7 |
| `frc/robot/subsystems/ExampleSubsystem.java` | Other fixes and strategy | 0 | 44 |
| `frc/robot/subsystems/ExampleSubsystem.java` | Simulation | 0 | 3 |
| `frc/robot/subsystems/HoodSubsystem.java` | Other fixes and strategy | 39 | 47 |
| `frc/robot/subsystems/HoodSubsystem.java` | Simulation | 6 | 27 |
| `frc/robot/subsystems/IntakeSubsystem.java` | Other fixes and strategy | 129 | 203 |
| `frc/robot/subsystems/IntakeSubsystem.java` | Simulation | 25 | 50 |
| `frc/robot/subsystems/KrakenMotorSubsystem.java` | Other fixes and strategy | 0 | 58 |
| `frc/robot/subsystems/KrakenMotorSubsystem.java` | Simulation | 0 | 28 |
| `frc/robot/subsystems/PrecisionDrive.java` | Other fixes and strategy | 26 | 0 |
| `frc/robot/subsystems/PrecisionModuleAngleHoldRequest.java` | Other fixes and strategy | 43 | 0 |
| `frc/robot/subsystems/ShooterSubsystem.java` | Other fixes and strategy | 41 | 89 |
| `frc/robot/subsystems/ShooterSubsystem.java` | Simulation | 12 | 29 |
| `frc/robot/subsystems/SmartDashboardSubsystem.java` | Other fixes and strategy | 1 | 5 |
| `frc/robot/subsystems/SpindexerSubsystem.java` | Other fixes and strategy | 34 | 27 |
| `frc/robot/subsystems/SpindexerSubsystem.java` | Simulation | 6 | 24 |
| `frc/robot/subsystems/TransferSubsystem.java` | Other fixes and strategy | 34 | 169 |
| `frc/robot/subsystems/TransferSubsystem.java` | Simulation | 6 | 28 |
| `frc/robot/subsystems/TurretSubsystem.java` | Other fixes and strategy | 89 | 327 |
| `frc/robot/subsystems/TurretSubsystem.java` | Simulation | 15 | 49 |
| `frc/robot/subsystems/package-info.java` | Other fixes and strategy | 7 | 0 |
| `frc/robot/subsystems/vision/LocalizationBootstrap.java` | Vision migration | 85 | 0 |
| `frc/robot/subsystems/vision/PoseJitterAccumulator.java` | Vision migration | 139 | 0 |
| `frc/robot/subsystems/vision/SingleTagTrigSolver.java` | Vision migration | 86 | 0 |
| `frc/robot/subsystems/vision/Vision.java` | Vision migration | 583 | 0 |
| `frc/robot/subsystems/vision/VisionFactory.java` | Vision migration | 61 | 0 |
| `frc/robot/subsystems/vision/VisionIO.java` | Vision migration | 87 | 0 |
| `frc/robot/subsystems/vision/VisionIOPhotonVision.java` | Vision migration | 161 | 0 |
| `frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java` | Simulation | 69 | 0 |
| `frc/robot/subsystems/vision/VisionPolicy.java` | Vision migration | 289 | 0 |
| `frc/robot/subsystems/vision/package-info.java` | Vision migration | 7 | 0 |
