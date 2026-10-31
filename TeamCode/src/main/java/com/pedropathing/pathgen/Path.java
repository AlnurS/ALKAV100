package com.pedropathing.pathgen;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.BezierLine;

public class Path extends org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Path {
    public Path(BezierLine line) {
        super(line);
    }

    public Path(BezierCurve curve) {
        super(curve);
    }

    public Path(Pose startPose, Pose endPose) {
        super(startPose, endPose);
    }
}
