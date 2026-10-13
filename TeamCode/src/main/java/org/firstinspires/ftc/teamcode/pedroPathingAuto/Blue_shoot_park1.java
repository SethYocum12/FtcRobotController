package org.firstinspires.ftc.teamcode.pedroPathingAuto;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;

@Autonomous(name = "Blue Shoot Park1", group = "Autonomous")
    public class Blue_shoot_park1 extends LinearOpMode {

        IMU_setup the_imu = new IMU_setup();
        private Follower follower;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(82.4949, 132.9043, 90);
        private final Pose path1Start = poseFactory.of(82.4949, 132.9043, 270);
        private final Pose path1 = poseFactory.of(82.4949, 103.8385, 270);
        private final Pose point2 = poseFactory.of(129.0391, 46.3025, 180);
        private final Pose point2Control1 = poseFactory.of(128.9023, 115.8992, 0);

        // Autonomous routine
        public Command autoRoutine() {
            return sequential(
                    follow(follower, path1()),
                    follow(follower, path2())
            );
        }

        @Override
        public void runOpMode() {
            the_imu.init(hardwareMap);

            if (the_imu != null){
                the_imu.resetYaw();
            }

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

        public Path path2() {
            return curve(path1, point2Control1, point2).linear(path1, point2);
        }
    }

