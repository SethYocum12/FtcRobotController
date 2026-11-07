package org.firstinspires.ftc.teamcode.mechanisms;

public class ShotSolution {
    public final double turn;
    public final double targetVelocity;
    public final boolean ready;
    public final String status;
    public final double aimErrorDegrees, leadDegrees, sidewaysSpeed, normalSpeed;

    public ShotSolution(double turn, double targetVelocity,
                        boolean ready, String status) {
        this(turn, targetVelocity, ready, status, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    public ShotSolution(double turn, double targetVelocity, boolean ready, String status,
                        double aimErrorDegrees, double leadDegrees, double sidewaysSpeed, double normalSpeed) {
        this.turn = turn;
        this.targetVelocity = targetVelocity;
        this.ready = ready;
        this.status = status;
        this.aimErrorDegrees = aimErrorDegrees;
        this.leadDegrees = leadDegrees;
        this.sidewaysSpeed = sidewaysSpeed;
        this.normalSpeed = normalSpeed;
    }
}
