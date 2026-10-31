package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseFactory;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

/**
 * Standard Autonomous OpMode using Pedro Pathing PathChain.
 * 1. Едем прямо из (31, 8) в (31, 25) под углом 90°
 * 2. Поворачиваем на месте с 90° на 0° в точке (31, 25)
 * 3. Едем прямо из (31, 25) в (35, 25) под углом 0°
 */
@Autonomous(name = "Main Autonomous (67)", group = "Autonomous")
public class BasicAuto extends OpMode {

    private Follower follower;
    private Timer opModeTimer;

    // Подсистемы робота
    private final Intake intake = new Intake();
    private final Stopper stopper = new Stopper();
    private final Shooting shooting = new Shooting();

    // Фабрика поз (угол в градусах)
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (Заданные координаты)
    private final Pose start1      = poseFactory.of(31, 8, 90);   // Старт
    private final Pose path8       = poseFactory.of(31, 25, 90);  // Конец первого движения (прямо)
    private final Pose path8Turned = poseFactory.of(31, 25, 0);   // Поворот на месте с 90° до 0°
    private final Pose point3      = poseFactory.of(45, 25, 0);   // Конец отдельного движения после поворота

    // Единственная цепочка путей (PathChain)
    private PathChain autoPathChain;

    public Path line(Pose start, Pose end) {
        return new Path(new BezierLine(new Point(start), new Point(end)));
    }

    public void buildPaths() {
        // Создаем единую неразрывную цепочку отрезков
        autoPathChain = new PathChain(
            // Отрезок 1: движение прямо вперед от start1 к path8 под углом 90°
            line(start1, path8).constant(path8),
            
            // Отрезок 2: чистый поворот на месте в точке (31, 25) с 90° на 0°
            line(path8, path8Turned).linear(path8.getHeading(), path8Turned.getHeading()),
            
            // Отрезок 3: движение прямо от path8Turned (31, 25) к point3 (35, 25) под углом 0°
            line(path8Turned, point3).constant(point3)
        );
    }

    @Override
    public void init() {
        opModeTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(start1);
        follower.setMaxPower(0.6); // 60% мощности
        buildPaths();

        intake.init(hardwareMap);
        stopper.init(hardwareMap);
        shooting.init(hardwareMap);

        telemetry.addData("Status", "Main Autonomous Initialized & Ready!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.setStartingPose(start1);
        follower.update();

        telemetry.addData("Status", "INIT LOOP - Calibrating Pose...");
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        follower.setStartingPose(start1);
        
        // Запускаем единую цепочку движения (PathChain)
        follower.followPath(autoPathChain, true);
    }

    @Override
    public void loop() {
        // Обновляем состояние фолловера и подсистем
        follower.update();
        intake.update();
        shooting.update();

        // Телеметрия
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Current X", "%.2f in", follower.getPose().getX());
        telemetry.addData("Current Y", "%.2f in", follower.getPose().getY());
        telemetry.addData("Current Heading", "%.2f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("OpMode Time", "%.2f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
