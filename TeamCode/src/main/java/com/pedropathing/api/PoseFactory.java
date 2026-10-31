package com.pedropathing.api;

import com.pedropathing.math.Pose;

public class PoseFactory {
    private boolean useDegrees = true;

    public static PoseFactory degrees() {
        PoseFactory factory = new PoseFactory();
        factory.useDegrees = true;
        return factory;
    }

    public static PoseFactory radians() {
        PoseFactory factory = new PoseFactory();
        factory.useDegrees = false;
        return factory;
    }

    public Pose of(double x, double y, double heading) {
        if (useDegrees) {
            return new Pose(x, y, Math.toRadians(heading));
        } else {
            return new Pose(x, y, heading);
        }
    }

    public Pose of(double x, double y) {
        return new Pose(x, y, 0.0);
    }
}
