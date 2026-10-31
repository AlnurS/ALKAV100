package com.pedropathing.api;

import com.pedropathing.math.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.paths.Path;

public class Paths {
    public static Path line(Pose start, Pose end) {
        return new Path(new BezierLine(new Point(start), new Point(end)));
    }

    public static Path line(org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose start, org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose end) {
        return new Path(new BezierLine(new org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point(start), new org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point(end)));
    }

    public static Path curve(Pose start, Pose control, Pose end) {
        return new Path(new BezierCurve(new Point(start), new Point(control), new Point(end)));
    }

    public static Path curve(org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose start, org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose control, org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose end) {
        return new Path(new BezierCurve(new org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point(start), new org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point(control), new org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point(end)));
    }
}
