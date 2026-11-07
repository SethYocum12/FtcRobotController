package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Experimental home-board test. A deliberate CENTER capture defines this run's frame.
 * Reference assumes the rear shooter is manually aimed at the target during capture.
 * Uses Pinpoint odometry for live bearing/lead; no AprilTag is required.
 */
public final class PracticeMovingShotPlanner {
    public static final double TARGET_SPEED = 1300;
    private static final double FLIGHT_SECONDS = 0.46; // Provisional single-shot estimate.
    private static final double SHOOTER_FORWARD = -6.5;
    private static final double HALF_LINE = 12, MAX_CROSS = 3, MAX_SPEED = 15;
    // Test margin for stopping feed near the boundary, NOT a measured feed delay.
    private static final double BOUNDARY_LOOKAHEAD = 0.25;
    private static final double MAX_NORMAL_SPEED = 3, MAX_TURN_RATE = Math.toRadians(30);
    private static final double MAX_ACCELERATION = 40, STEADY_SECONDS = 0.08;
    private final AimAssist aim;
    private final servos_and_shooters shooter;
    private final ElapsedTime clock = new ElapsedTime();
    private boolean armed, stable;
    private double cx, cy, lineX, lineY, tx, ty, previousTime = Double.NaN;
    private double previousVx, previousVy, stableSince;
    private String referenceStatus = "At CENTER: manually aim, shooter OFF, click right stick";

    public PracticeMovingShotPlanner(AimAssist aim, servos_and_shooters shooter) {
        this.aim = aim; this.shooter = shooter;
    }

    public boolean isArmed() { return armed; }
    public String referenceStatus() { return referenceStatus; }
    public void resetMotion() { stable = false; previousTime = Double.NaN; }
    public void clear() {
        armed = false; resetMotion();
        referenceStatus = "At CENTER: manually aim, shooter OFF, click right stick";
    }

    private static boolean finite(Pose p, Velocity v) {
        return p != null && v != null && Double.isFinite(p.x()) && Double.isFinite(p.y())
                && Double.isFinite(p.heading()) && Double.isFinite(v.vx)
                && Double.isFinite(v.vy) && Double.isFinite(v.omega);
    }

    public void capture(Pose p, Velocity v) {
        clear();
        if (shooter.isShooterEnabled()) {
            referenceStatus = "Turn shooter OFF before setting CENTER"; return;
        }
        if (!finite(p,v) || Math.hypot(v.vx,v.vy) > 1 || Math.abs(v.omega) > Math.toRadians(5)) {
            referenceStatus = "Stop moving before setting CENTER"; return;
        }
        cx = p.x(); cy = p.y();
        lineX = -Math.sin(p.heading()); lineY = Math.cos(p.heading());
        // Camera: 5 in behind center, 0.5 in left. Tape floor range: 39 in.
        double targetBehind = 5 + Math.sqrt(39 * 39 - 0.5 * 0.5);
        tx = cx - targetBehind * Math.cos(p.heading());
        ty = cy - targetBehind * Math.sin(p.heading());
        armed = true;
        referenceStatus = "CENTER set: live Pinpoint aim, +/-12 in; right stick cancels";
    }

    private ShotSolution blocked(String status) {
        resetMotion();
        return new ShotSolution(0, TARGET_SPEED, false, status);
    }

    public ShotSolution evaluate(Pose p, Velocity v) {
        if (!armed) return blocked("Set CENTER first");
        if (!finite(p,v)) return blocked("Invalid odometry");
        double along = (p.x()-cx)*lineX + (p.y()-cy)*lineY;
        double cross = -(p.x()-cx)*lineY + (p.y()-cy)*lineX;
        double side = v.vx*lineX + v.vy*lineY;
        double normal = -v.vx*lineY + v.vy*lineX;
        if (Math.abs(along)>HALF_LINE || Math.abs(cross)>MAX_CROSS
                || Math.abs(along+side*BOUNDARY_LOOKAHEAD)>HALF_LINE
                || Math.abs(cross+normal*BOUNDARY_LOOKAHEAD)>MAX_CROSS)
            return blocked(String.format(java.util.Locale.US,
                    "Near boundary: sideways %.1f/12, forward %.1f/3 in; move toward CENTER", along,cross));
        double ox = SHOOTER_FORWARD*Math.cos(p.heading());
        double oy = SHOOTER_FORWARD*Math.sin(p.heading());
        double dx = tx-p.x()-ox, dy = ty-p.y()-oy;
        ShotMotion.Lead lead = ShotMotion.calculate(dx,dy,v.vx-v.omega*oy,
                v.vy+v.omega*ox,FLIGHT_SECONDS,0);
        if (lead == null) return blocked("Invalid lead");
        double error = Math.toDegrees(wrap(lead.fieldBearingRadians-p.heading()-Math.PI));
        double leadDegrees = Math.toDegrees(wrap(lead.fieldBearingRadians-Math.atan2(dy,dx)));
        double now = clock.seconds(), dt = now-previousTime;
        boolean steady = Double.isFinite(previousTime) && dt>0 && dt<=0.25
                && Math.hypot(v.vx-previousVx,v.vy-previousVy)/dt<=MAX_ACCELERATION;
        previousTime=now; previousVx=v.vx; previousVy=v.vy;
        boolean slow = Math.abs(side)<=MAX_SPEED && Math.abs(normal)<=MAX_NORMAL_SPEED
                && Math.abs(v.omega)<=MAX_TURN_RATE;
        if (!steady || !slow) stable=false;
        else if (!stable) { stable=true; stableSince=now; }
        String status;
        boolean ready=false;
        if (Math.abs(side)>MAX_SPEED) status=String.format(java.util.Locale.US,"Sideways %.1f > 15 in/s: slow down",Math.abs(side));
        else if (Math.abs(normal)>MAX_NORMAL_SPEED) status="Too much forward/back movement (max 3 in/s)";
        else if (Math.abs(v.omega)>MAX_TURN_RATE) status="Turning too fast (max 30 deg/s)";
        else if (!shooter.isShooterEnabled()) status="Press Y: shooter ON";
        else if (!Double.isFinite(shooter.getShooterTargetVelocity())
                || Math.abs(shooter.getShooterTargetVelocity()-TARGET_SPEED)>0.5
                || !shooter.isShooterReady()) status="Waiting for 1300 speed / recovery";
        else if (Math.abs(error)>3) status=String.format(java.util.Locale.US,"Aiming: %.1f deg error (need within 3)",error);
        else if (!stable || now-stableSince<STEADY_SECONDS) status="Changing speed: hold stick steady briefly";
        else { ready=true; status="MOVING PRACTICE READY"; }
        return new ShotSolution(aim.calculateTurn(error),TARGET_SPEED,ready,status,
                error,leadDegrees,side,normal);
    }

    private static double wrap(double radians) { return Math.atan2(Math.sin(radians),Math.cos(radians)); }
}
