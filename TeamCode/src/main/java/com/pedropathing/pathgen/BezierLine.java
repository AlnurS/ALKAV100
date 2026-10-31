package com.pedropathing.pathgen;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point;

public class BezierLine extends org.firstinspires.ftc.teamcode.pedroPathing.pathgen.BezierLine {
    public BezierLine(Point start, Point end) {
        super(start, end);
    }

    public BezierLine(Pose start, Pose end) {
        super(start, end);
    }

    public BezierLine(com.pedropathing.math.Pose start, com.pedropathing.math.Pose end) {
        super(new Pose(start.getX(), start.getY(), start.getHeading()), new Pose(end.getX(), end.getY(), end.getHeading()));
    }
}
