package org.firstinspires.ftc.teamcode.mechanisms;

/** Shares the last started autonomous alliance with TeleOp in this app process. */
public final class AllianceMemory {
    public enum Alliance { UNKNOWN, RED, BLUE }

    private static volatile Alliance alliance = Alliance.UNKNOWN;

    private AllianceMemory() { }

    public static void remember(boolean redAlliance) {
        alliance = redAlliance ? Alliance.RED : Alliance.BLUE;
    }

    public static Alliance get() {
        return alliance;
    }
}
