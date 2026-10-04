# Bringing existing robot code into this lab

1. Keep the original project as the current robot baseline. The robot-build folder here is an independent snapshot, not a link to that project.
2. Pick one simple existing autonomous route. Put its path geometry and sequencing into shared/src. Make its follower creation go through LearningRobot.create(hardwareMap). Remove the old duplicate class from the LAB robot-build source tree if retaining the same package/class name.
3. Run it in MecDynamic with sim-profile, then build the exact same shared source using Build-Robot.cmd. The desktop uses approximate FTC SDK classes; Android uses the real SDK. Both builds are required to catch unsupported APIs.
4. Keep motor names/directions, Pinpoint offsets and pod choices, robot dimensions, and controller tuning in the environment-specific configuration. The initial robot profile delegates to robot-build's existing pedroPathing.Constants; simulator configuration is independent.
5. For intake/indexer/shooter, introduce a small mechanism interface: real hardware implementation for robot, simulated/logging implementation for desktop. Add one mechanism at a time. Existing whole-robot OpModes are not automatically desktop compatible, particularly camera/Android/vendor features.
6. Before the first physical test, verify actual pod offsets/units, encoder directions, and heading; tune the real robot. Start with a short path in a clear space. Only then deliberately deploy the lab robot-build app. No deployment was performed during setup.
7. Once both environments are useful, review and port the shared-source/profile layout back into the team repository in its own change. Do not copy mock SDK classes or simulator configuration into the robot app.

Success criterion for the first father/son session: Seth predicts the endpoint, changes the distance himself, explains the telemetry result, and identifies the configuration that differs on the real robot.
