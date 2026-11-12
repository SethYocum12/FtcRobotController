package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.function.DoubleSupplier;

/** Shared autonomous rule: tag-triggered feeding, then depart at 20 seconds after START. */
public final class AutonomousShooting {
    public static final double DEPART_AT_SECONDS = 20.0;
    public static final double SHOOTER_READY_RATIO = 0.97;

    private final servos_and_shooters mechanisms;
    private final LIMELIGHT_setup vision;
    private final DoubleSupplier autoSeconds;
    private final boolean redAlliance;
    private final double targetVelocity;
    private final double indexerPower;
    private boolean feeding;
    private int selectedGroupStart = -1;
    private double feedStartSeconds;
    private double completedFeedSeconds;
    private String status = "Waiting for START";

    /** The supplied clock must be reset immediately after waitForStart(). */
    public AutonomousShooting(servos_and_shooters mechanisms, LIMELIGHT_setup vision,
                              DoubleSupplier autoSeconds, boolean redAlliance,
                              double targetVelocity, double indexerPower) {
        this.mechanisms = mechanisms;
        this.vision = vision;
        this.autoSeconds = autoSeconds;
        this.redAlliance = redAlliance;
        this.targetVelocity = targetVelocity;
        this.indexerPower = indexerPower;
    }

    /** The deadline also interrupts an approach path that has not completed by 20 seconds. */
    public Command approachAndShoot(Follower follower, Path approach, Pose shootingPose) {
        return sequential(
                sequential(
                        follow(follower, approach),
                        // Passive BRAKE avoids repeated endpoint corrections while feeding.
                        instant(follower::stop),
                        new CommandBuilder()
                                .setStart(this::start)
                                .setExecute(this::update)
                                .setDone(this::deadlineReached)
                                .setEnd(reason -> stop())
                ).raceWith(waitUntil(this::deadlineReached)),
                instant(() -> {
                    stop();
                    follower.stop();
                })
        );
    }

    public boolean deadlineReached() {
        return autoSeconds.getAsDouble() >= DEPART_AT_SECONDS;
    }

    /** Call after START, not in the constructor: OpMode scanning must not pick an alliance. */
    public void rememberAlliance() {
        AllianceMemory.remember(redAlliance);
    }

    private void start() {
        feeding = false;
        selectedGroupStart = -1;
        completedFeedSeconds = 0;
        if (deadlineReached()) {
            stop();
            return;
        }
        status = "Waiting for alliance tag and shooter speed";
        mechanisms.indexerthingy(false, 0);
        mechanisms.shooterthingy(true, targetVelocity);
    }

    private void update() {
        if (deadlineReached()) {
            stop();
            return;
        }
        if (!feeding) {
            int audienceStart = redAlliance ? 34 : 38;
            int oppositeStart = redAlliance ? 30 : 42;
            boolean audienceVisible = groupVisible(audienceStart);
            boolean oppositeVisible = groupVisible(oppositeStart);
            boolean ready = mechanisms.getShooterVelocity() >= targetVelocity * SHOOTER_READY_RATIO
                    && mechanisms.isShooterReady();
            if (audienceVisible == oppositeVisible || !ready) return;

            selectedGroupStart = audienceVisible ? audienceStart : oppositeStart;
            feedStartSeconds = autoSeconds.getAsDouble();
            feeding = true;
            status = "Feeding until 20 seconds; tag trigger latched";
        }
        // Always update the feed timer, including speed dips, so a pulse cannot resume halfway through.
        mechanisms.feedIndexerWithRecovery(indexerPower);
        status = mechanisms.getIndexerFeedStatus();
    }

    private boolean groupVisible(int firstId) {
        for (int id = firstId; id < firstId + 4; id++) {
            if (vision.hasTag(id)) return true;
        }
        return false;
    }

    public void stop() {
        if (feeding) completedFeedSeconds = getFeedSeconds();
        feeding = false;
        mechanisms.indexerthingy(false, 0);
        mechanisms.shooterthingy(false, 0);
        status = deadlineReached() ? "20-second cutoff: moving to next path" : "Stopped";
    }

    public String getStatus() { return status; }

    public String getSelectedCell() {
        return selectedGroupStart < 0 ? "None"
                : (redAlliance ? "Red " : "Blue ") + selectedGroupStart + "-" + (selectedGroupStart + 3);
    }

    public double getFeedSeconds() {
        return feeding ? Math.max(0, autoSeconds.getAsDouble() - feedStartSeconds)
                : completedFeedSeconds;
    }

    public boolean selectedCellVisible() {
        return selectedGroupStart >= 0 && groupVisible(selectedGroupStart);
    }
}
