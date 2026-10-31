package com.pedropathing.paths;

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

    public Path(com.pedropathing.math.Pose startPose, com.pedropathing.math.Pose endPose) {
        super(new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()), new Pose(endPose.getX(), endPose.getY(), endPose.getHeading()));
    }

    @Override
    public Path linear(double startHeading, double endHeading) {
        super.linear(startHeading, endHeading);
        return this;
    }

    public Path linear(com.pedropathing.math.Pose startPose, com.pedropathing.math.Pose endPose) {
        setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        return this;
    }

    @Override
    public Path reverseTangent() {
        super.reverseTangent();
        return this;
    }
}
