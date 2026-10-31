package com.pedropathing.pathgen;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Point extends org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point {
    public Point(double x, double y) {
        super(x, y);
    }

    public Point(Pose pose) {
        super(pose);
    }

    public Point(com.pedropathing.math.Pose pose) {
        super(pose.getX(), pose.getY());
    }
}
