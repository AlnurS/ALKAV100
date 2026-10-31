package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

/**
 * Autonomous OpMode for BLUE Alliance using Pedro Pathing (Part 1 only).
 */
@Autonomous(name = "Basic Auto BLUE Alliance", group = "Autonomous")
public class BasicAutoBlue extends OpMode {

    private Follower follower;
    private Timer opModeTimer;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =========================================================================
    // POSES & CONTROL POINTS (Точки и контрольная точка кривой Безье)
    // =========================================================================
    // ЧАСТЬ 1: Движение по дуге от start1 к point1
    private final Pose start1         = poseFactory.of(55, 8, 90);       // Старт: (55, 8, 90°)
    private final Pose point1Control1 = poseFactory.of(42.5, 44.8, 90);  // Контрольная точка 1
    private final Pose point1         = poseFactory.of(10, 47, 180);     // Конец части 1: (10, 47, 180°)

    // Цепочка путей (PathChain)
    private PathChain autoPathChain;

    // =========================================================================
    // PATH GENERATION (Создание траектории)
    // =========================================================================
    public Path path1() {
        // Часть 1: Дуга Безье с плавным линейным поворотом корпуса от 90° к 180°
        return curve(start1, point1Control1, point1).linear(start1, point1);
    }

    public void buildPaths() {
        // Задаем только Часть 1
        autoPathChain = new PathChain(
            path1()
        );
    }

    @Override
    public void init() {
        opModeTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(start1);
        follower.setMaxPower(0.6); // 60% мощности
        buildPaths();

        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Status", "BLUE Autonomous Initialized & Ready!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.setStartingPose(start1);
        follower.update();

        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        follower.setStartingPose(start1);
        
        // Запуск Части 1
        follower.followPath(autoPathChain, true);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("OpMode Time", "%.2f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
