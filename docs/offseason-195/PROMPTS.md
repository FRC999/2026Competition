# Human prompt record

## 2026-09-30 — initial request (verbatim)

We are FRC team participating in the First Robotics competition. We might decide to go into the 195 Open off-season competition. But for that we need to improve the code for our 2026 robot.
The main Github repo for our 2026 bot is [https://github.com/FRC999/2026Competition](https://github.com/FRC999/2026Competition)

The branches that we can go with are either the ones that have a word "Houston" in them or the one that has "Worlds" in it. I do not recall which one is the latest one, so you should check the dates.
So, determine which one is the one we used at the Worlds, and that one is probably the one we should use as the one we will modify for this competition.

After the season was over, we worked quite a bit to improve our driving. And so there is a repo [https://github.com/FRC999/2027Prototyping](https://github.com/FRC999/2027Prototyping) that has all of our latest vision improvements.

So, if we are going to this off-season competition, the team made the decision to replace cameras from the limelight with the cameras that will use Photon Vision on Orange Pi, and we are also not going to use Quest Nav. The mounts for cameras will be the hard mounts made of aluminum, so they will not break or bend, and the cameras will use the new shells we developed for them, so the cameras will be protected from flying balls, essentially, as well. What I want to do is to see if we can retrofit our latest vision improvements into another branch of our 2026 competition code to see that we can drive better and more precisely. The main problem was that all our driving before essentially was based on Path Planner, and we never really arrived where we needed to be. We want to improve precision of driving and also precision of aiming. So that's something we only started in the vision 2027 prototyping repo. However, that may be something you can help me with to improve and test. The idea is this: we probably will use two to four cameras, and you can tell us what angles they probably should be used at if the cameras will be installed on top or close to the top of the drive modules. And we can probably just install maybe four cameras or so, or maybe two cameras, whatever, on top of the module to see whether that will work. We will start with definitely two cameras. You can tell us what angles and where they need to be pointed, and we also need a way to find out their exact angles, in other words, to be able to calibrate them properly. We need to improve the driving of trajectories and aiming and make sure the turret is working correctly. The turret movement was designed with AI a long time ago. It was between Claude and Codex, and it was a lot older models. So it was models that were essentially at least six months old or more. We are using the latest models now, so I expect maybe you can improve the algorithms and improve the way how we do things. You can analyze the code, you can analyze the prompts and whatever else we store. And the goal is to both drive and aim better.

You can close both repos that I mentioned above into folders under S:\Projects\MechaRAMS.

The final product should be a branch of the "2026Competition" competition repo. You can call it "OffSeason-195".

So the idea of improvement is this. We want to merge all of our advances that we've done with vision into this new branch we are creating. And also we are replacing cameras. So we are going to get rid of limelights, and we are going to replace them with at least two, or maybe up to four cameras that will be driven by Photon Vision connected to Orange Pi. We definitely need routines to be able to calibrate the cameras quickly. In other words, both the calibration of the yaw, or rather figuring out, is it a proper yaw, pitch, and roll, and also figuring out the offsets from the center of rotation of the bot. We can try measuring them, but it's not going to very precisely. We are not going to do that during competition. We can do it before that, but I want to try and see if we can limit the calibration time less than 30 minutes per camera. We have done it before. I will need detailed instructions to be added to the repo as documents, or created them somewhere, but they should not be in the code, obviously. They should not be deployed in any way. But I'm okay actually to making them as part of the repo, somewhere in a documentation-type directory. The other thing that I wanted to do, again, is to see what else needs to be done for those cameras. So I really need step-by-step instructions people can understand for both installation as well as calibration. I need to be able to create a custom field for the calibration, so we can do it with two tags that we do in our current vision calibration, at least to determine the pitch, roll, and yaw, as well as the offsets, and then being able to reload the field for the actual competition. I also want to be able to convert all of this into skills, so make sure to create skill files, and you can save them into the repo as well for everything you are doing. It also would be a good idea to create the files that will save the current state of the AI-type sessions, so we don't run out of tokens. Or if we do run out of tokens, you at least remember where you are, or if something dies, we remember where you are. And also create a separate file with all the prompts. Those files will be in the MD format, so it will be easily visible or accessible from the GitHub. And the skill files that you need to create should be usable from both Claude as well as Codex, which may mean that you may need to create two different files. And you need to make sure that you need to update them every time. Be truthful and direct. Definitely do not guess. If you have more questions, definitely stop and ask before just starting doing things. However, I do want you to be as autonomous as possible, in the sense that if you don't have questions, don't just stop and tell me what you've done so far. If you have more things that you can do, just continue doing things until you're either completely done, or have questions that you need answers to.

## 2026-09-30 — GitHub authorization (verbatim)

> Also, if you need access to those repos, just let me know and I can set up access with my account, so you can definitely propagate the changes as commits into the repository properly. So yes, I do want that to become part of the same repo, just as I said, a different branch.

## 2026-09-30 — hardware answers (verbatim)

- Cameras/processor: "Arducam OV9782; Orange Pi 5 Plus (I have two of those if needed)"
- Mechanisms changed: "unchanged"

## 2026-09-30 — audit request (verbatim)

> Also make sure to analyze our 2026 code that we have now for issues or logic errors or anything like that. We do have specific way how we want it to point the turret with the offsets and everything. So just ask us how exactly we're gonna do that.

## 2026-09-30 — turret answers (verbatim)

- Geometry: "we're not changing location of turret or anything else except cameras. So I would presume those locations to be correct"
- Unreachable target/travel: "I would rely on your advice here. Physical travel - what you said is good numbers."
- Moving shots: "Ideally if we drive while shooting, it would predict advance offset based on robot's direction and speed, so preemptively aim a little \"before\" the center. The actual angle would depend on the robot speed and turret's capabilities, I assume. Most shots are done stationary, though."

## 2026-09-30 — camera configuration and library updates (verbatim)

> Note that the Arducams will be at different offsets and roll, pitch, yaw as in LimeLight. They are smaller, so we are more flexible where to put them in. We just need a way to calibrate them. So just keep space for variables, essentially. Put them now at the same location, but flag those locations so we can easily change them. Note also that we can update libraries with the latest C2026 for CTR path planner and whatever else happened later.

## 2026-09-30 — revised mount proposal request (verbatim; supersedes old LL coordinates)

> We do not care about the old mount points. But they were installed to the left and right of the turret. So, near the bumper, roughly 6 inch to the left and right of the turret centerline. But here assume we can mount them at the back line of the bot near bumper pretty much anywhere, and they probably should have a pitch. Let's say they will be 12 inch above ground as the starting point, and just at the bot perimeter. You come up with proposed pitch and yaw. I assume you want 0 roll.

## 2026-09-30 — turret limit clarification (verbatim)

> Actually turret physically CAN rotate more. We did that to comply with competition rules. If we rotate too much, one of the motors goes slightly outside the bot perimeter. So, we need to control its rotation in software.

## 2026-09-30 — AdvantageKit authorization (verbatim)

> Also, we are okay to implement advantage kit for all the testing. We have not really used that in our 2026 code, but may not be a bad idea to put that in this code.
