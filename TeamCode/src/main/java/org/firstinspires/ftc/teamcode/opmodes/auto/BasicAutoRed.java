package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseFactory;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

/**
 * Autonomous OpMode for RED Alliance using Pedro Pathing.
 */
@Autonomous(name = "Basic Auto RED Alliance", group = "Autonomous")
public class BasicAutoRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState = 0;

    // Подсистемы робота
    private final Intake intake = new Intake();
    private final Stopper stopper = new Stopper();
    private final Shooting shooting = new Shooting();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start1 = poseFactory.of(55, 8, 90);
    private final Pose point1 = poseFactory.of(45, 36, 90);
    private final Pose point2 = poseFactory.of(55, 36, 180);

    // Paths
    private Path pathSegment1, pathSegment2;

    public Path line(Pose start, Pose end) {
        return new Path(new BezierLine(new Point(start), new Point(end)));
    }

    public Path path1() {
        return line(start1, point1).linear(start1, point1);
    }

    public Path path2() {
        return line(point1, point2).reverseTangent();
    }

    public void buildPaths() {
        pathSegment1 = path1();
        pathSegment2 = path2();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 1. Движение от start1 (55, 8, 90°) к point1 (50, 36, 90°)
                follower.followPath(pathSegment1, true);
                setPathState(1);
                break;

            case 1: // 2. Прибыли в point1 -> Поворот к point2 (50, 36, 180°)
                if (!follower.isBusy()) {
                    follower.followPath(pathSegment2, true);
                    setPathState(2);
                }
                break;

            case 2: // 3. Завершение пути после поворота
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case -1: // Финиш
                follower.stopMotors();
                intake.stop();
                shooting.stop();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(start1);
        follower.setMaxPower(0.6);
        buildPaths();

        intake.init(hardwareMap);
        stopper.init(hardwareMap);
        shooting.init(hardwareMap);

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "RED Autonomous Initialized & Ready!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.setStartingPose(start1);
        follower.update();

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        follower.setStartingPose(start1);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        intake.update();
        shooting.update();

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("OpMode Time", "%.2f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
