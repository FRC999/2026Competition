# OffSeason-195: start here

This branch retrofits the **2026 competition robot**, retaining its drivetrain and mechanism geometry.
It replaces the active Limelight/Quest localization with PhotonVision, adds a measured-motion finish
to stopping paths, and corrects several turret, hood and shooting-readiness defects.

**It is a software candidate for robot testing, not a calibrated competition release.** The shipped
real-camera transforms are deliberately unmeasured. Those cameras can produce calibration data but
cannot correct odometry until calibrated and the field-layout acknowledgment is set. Desktop tests
do not establish centimeter accuracy, shot percentage, or a safe mechanism envelope.

## Student/operator sequence

1. [Install and focus the cameras](installation.md). Start with one rear camera on each Orange Pi 5 Plus.
2. [Calibrate intrinsics and all six mount offsets](calibration.md). Prepare the survey area once;
   the recurring workflow targets less than 30 minutes per camera.
3. [Restore and verify the competition field](calibration.md#return-to-the-competition-field).
4. [Run the acceptance tests](testing.md), beginning with disabled checks and low-speed motion.
5. [Read the algorithm audit](audit.md) before tuning or enabling match autos.

## Developers

Robot project: `2026Competition/`. Documentation, calibration tools, session records and skills are
outside that project's `src/` tree. Its Gradle build archives `src/` in the JAR, so do not move these
documents into `src/main/resources` or `src/main/deploy`.

Use Java 17 from WPILib 2026. From the robot project:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat test robotSmoke build --console=plain
```

`robotSmoke` boots the complete robot in desktop simulation in a separate JVM, produces a WPILOG,
and checks vision reception and the stationary default auto. It does not deploy. For a workstation
whose Java trust store cannot validate the dependency server's certificate chain, this session used
the Windows root store (not disabled certificate verification):

```powershell
$env:JAVA_TOOL_OPTIONS = '-Djavax.net.ssl.trustStoreType=Windows-ROOT -Djavax.net.ssl.trustStore=NONE'
```

From repository root, `python -m unittest discover -s tools -v` checks the offline calibration fit.
See [testing.md](testing.md) for reports, limitations and reproducible field tests.

The selected base is `Houston---afternoon-Friday` (`6c4ecb4`, May 1, 2026), the latest robot-code
commit among the requested Houston/Worlds candidates. The other Houston branch has a later
documentation commit. This establishes a sensible Worlds-era source base; it does **not** prove
which binary was actually deployed at Worlds. The prototype source is pinned to `d20594a` in
`FRC999/2027Prototyping`; its chassis measurements and camera calibrations were not copied.

For AI continuation, read [SESSION_STATE.md](SESSION_STATE.md), [PROMPTS.md](PROMPTS.md), and the
appropriate skill in `.agents/skills/`. Matching Claude copies live in `.claude/skills/`.
