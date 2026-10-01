# Camera installation — 2026 robot

## What to install

Use the team's OV9782 cameras and two Orange Pi 5 Plus boards. Initially assign `back-left` to one
Pi and `back-right` to the other. This reduces USB sharing and gives each rear view its own processor.
The code supports up to four named cameras; add the optional front pair only after the rear pair
passes calibration and driving tests. Two cameras looking rearward do not provide all-around coverage.

The sensor model alone does not determine field of view: the lens matters. Inspect the image with
the actual lens, shell and bumper installed before finalizing a bracket. The values below are proposed
starting orientations, not measurements and not fit results.

| Camera | Proposed location | Lens height | Roll | Pitch | Yaw |
|---|---|---:|---:|---:|---:|
| back-left | Rear-left perimeter near bumper, clear of module and moving parts | 0.3048 m / 12 in | 0° | -15° | +165° |
| back-right | Rear-right perimeter near bumper, clear of module and moving parts | 0.3048 m / 12 in | 0° | -15° | -165° |
| front-left, optional | Front-left perimeter, unobstructed | Proposed 0.3048 m | 0° | -15° | +15° |
| front-right, optional | Front-right perimeter, unobstructed | Proposed 0.3048 m | 0° | -15° | -15° |

In robot coordinates, +X is forward, +Y left, +Z up. Positive yaw turns toward the robot's left.
**Negative WPILib pitch points the lens upward.** The rear pair points 15° outward from straight
backward and 15° upward. These are `Rotation3d(roll,pitch,yaw)` conventions; do not copy a user
interface's angle label without checking its coordinate convention. See the
[PhotonVision coordinate reference](https://docs.photonvision.org/en/v2026.3.4/docs/apriltag-pipelines/coordinate-systems.html).

All translations are from the drivetrain origin at floor level to the camera's optical center,
not to its housing screw. The exact X/Y positions are unknown and intentionally `null` in the
configuration. Do not insert the prototype robot's offsets. Aluminum brackets should constrain all
axes, and cable strain should not pull the camera. Check visibility across the shell opening at the
edges of the image, bumper occlusion, full turret travel and intake movement. Lock focus before
intrinsic calibration. Remounting requires a new extrinsic fit; refocusing requires new intrinsics.

## Power and network preparation

1. Label each camera, Pi, USB cable and mount with its camera name. Photograph the finished mount.
2. Have the electrical team provide a secured, regulated supply appropriate for the **5 Plus** board
   and attached USB load. Do not feed a board directly from unregulated robot battery voltage.
   Secure power, USB and Ethernet connections and provide cooling. Follow the regulator and board
   pinout specifications; generic Orange Pi diagrams are not proof of a 5 Plus pinout.
3. Connect both Pis to the robot Ethernet switch/radio. Establish the network on the bench first.
   Record assigned addresses; `10.9.99.11` and `10.9.99.12` are suggestions only if available.
   The expected team-999 roboRIO address is `10.9.99.2`; confirm it on the actual network.
4. Test both cameras concurrently while the robot is under electrical load during later operator-led
   testing. A bench image alone does not verify brownout resilience.

PhotonVision's [wiring guide](https://docs.photonvision.org/en/v2026.3.4/docs/quick-start/wiring.html)
and [network guide](https://docs.photonvision.org/en/v2026.3.4/docs/quick-start/networking.html)
provide the vendor-side setup reference.

## Image and camera setup

The branch uses WPILib **2026.2.1**, PhotonLib **v2026.3.4**, CTRE **26.3.0**, AdvantageKit
**26.0.2**, and PathPlanner **2026.1.2**. These are compatible 2026 releases checked on September 30,
2026. Do not install a 2027 alpha on either side of this system.

1. Back up any existing Pi configuration and calibration before replacing its image.
2. Download **`photonvision-v2026.3.4-linuxarm64_orangepi5plus.img.xz`** from the
   [v2026.3.4 release](https://github.com/PhotonVision/photonvision/releases/tag/v2026.3.4).
   Its release assets include the Plus image even though the quick-install table does not list it.
   Use the Plus image, not the generic `orangepi5` image.
3. Flash the intended removable card using the method in the
   [installation guide](https://docs.photonvision.org/en/v2026.3.4/docs/quick-start/quick-install.html),
   checking the destination device before writing. Boot on a bench supply. Confirm the UI reports
   **v2026.3.4**. For an already working image, PhotonVision's offline JAR update can update the
   application, but does not repair an incompatible underlying OS image.
4. Set team number **999**, unique hostnames such as `photon-back-left` / `photon-back-right`, and
   the actual network configuration. Open each Pi's UI using its address and port **5800**.
5. Connect one camera per Pi. Match the physical camera to the correct UI entry. Set its exact unique
   name to `back-left` or `back-right`. Cover one lens at a time to verify identities.
6. Pick an AprilTag pipeline and its final resolution. A useful initial OV9782 trial is 1280×800;
   verify the camera actually offers it. Measure achieved FPS, latency and processor temperature
   with both cameras active. Lower resolution only after recalibrating that resolution.
7. Focus on a tag at a representative working distance, then check near and far tags. Lock the lens
   without twisting it. Use a low enough exposure to prevent motion blur under field lighting;
   choose exposure from actual moving images, not an invented universal number.
8. Complete intrinsic calibration in [calibration.md](calibration.md). Enable 3D and MultiTag
   estimation. Keep individual single-tag estimation available because the robot has a single-tag
   fallback. Disable driver mode on cameras intended for localization.
9. Upload the identical chosen field JSON on **each Pi** and configure the robot using the field
   procedure. Camera extrinsics are applied in robot code to the raw field-to-camera result.
   Do not transform that result twice.
10. Export each Pi's settings, intrinsics, camera model/resolution, lens/focus notes, field JSON,
    version and capture date. Store the calibration evidence outside the robot `src/` directory.

Before driving, check `Vision/ConfigValid`, the selected profile, layout hash, camera connection
states and `FusionConfigured` in the log. `ConfigValid=true` does not mean the cameras are calibrated.

## Adding cameras three and four

Enable and calibrate `front-left` / `front-right` separately in `vision/cameras.json`. Use exact,
unique PhotonVision names; assign one additional camera to each Pi only after measuring aggregate
USB throughput, FPS, latency, temperature and voltage stability. Calibrate every physical camera at
its operating resolution. Do not copy another camera's intrinsics or offsets. Compare independent
camera errors before reducing noise factors; the default factor of 1.0 is provisional.
