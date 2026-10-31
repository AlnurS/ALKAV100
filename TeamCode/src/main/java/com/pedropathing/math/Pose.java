package com.pedropathing.math;

public class Pose extends org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose {
    public Pose(double x, double y, double heading) {
        super(x, y, heading);
    }

    public Pose(double x, double y) {
        super(x, y, 0.0);
    }

    public Pose() {
        super(0.0, 0.0, 0.0);
    }
}
