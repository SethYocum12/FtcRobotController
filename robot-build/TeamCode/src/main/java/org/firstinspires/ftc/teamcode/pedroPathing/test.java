package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;

@Autonomous(name = "AutoPath", group = "Autonomous")
    public class test extends LinearOpMode {

        IMU_setup the_imu = new IMU_setup();
        private Follower follower;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(132.3287, 8.4289, 90);
        private final Pose path1Start = poseFactory.of(132.3287, 8.4289, 180);
        private final Pose path1 = poseFactory.of(131.9907, 23.4805, 180);

        // Autonomous routine
        public Command autoRoutine() {
            if (the_imu != null){
                the_imu.resetYaw();;
            }
            return sequential(
                    follow(follower, path1())
            );
        }

        @Override
        public void runOpMode() {
            the_imu.init(hardwareMap);
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            waitForStart();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }
        }

        public Path path1() {
            return line(path1Start, path1).linear(path1Start, path1);
        }
    }

