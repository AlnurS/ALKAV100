package com.pedropathing.pathgen;

import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Path;

import java.util.List;

public class PathChain extends org.firstinspires.ftc.teamcode.pedroPathing.pathgen.PathChain {
    public PathChain(Path... paths) {
        super(paths);
    }

    public PathChain(List<Path> paths) {
        super(paths);
    }
}
