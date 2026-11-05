package org.firstinspires.ftc.teamcode.mechanisms;

public class ShotSolution {
    public final double turn;
    public final double targetVelocity;
    public final boolean ready;
    public final String status;

    public ShotSolution(double turn, double targetVelocity,
                        boolean ready, String status) {
        this.turn = turn;
        this.targetVelocity = targetVelocity;
        this.ready = ready;
        this.status = status;
    }
}
