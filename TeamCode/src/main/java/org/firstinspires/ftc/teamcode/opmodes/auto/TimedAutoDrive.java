package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ServoRamp;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

/**
 * Автономный режим: движение прямо по таймеру.
 * Робот едет прямо заданное количество секунд и останавливается.
 */
@Autonomous(name = "Timed Auto Straight", group = "Autonomous")
public class TimedAutoDrive extends LinearOpMode {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Stopper stopper = new Stopper();
    private final ServoRamp servoRamp = new ServoRamp();
    private final Shooting shooting = new Shooting();
//git remote add origin https://github.com/AlnurS/34155.SAN
    // Параметры движения по таймеру (настраиваются)
    public static double DRIVE_TIME_SECONDS = 0.30; // Время движения прямо в секундах
    public static double DRIVE_POWER = 0.5;        // Мощность моторов (50%)

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Безопасная инициализация подсистем
        try { drivetrain.init(hardwareMap); } catch (Exception ignored) {}
        try { stopper.init(hardwareMap); } catch (Exception ignored) {}
        try { servoRamp.init(hardwareMap); } catch (Exception ignored) {}
        try { shooting.init(hardwareMap); } catch (Exception ignored) {}

        telemetry.addData("Status", "Timed Auto Ready!");
        telemetry.addData("Drive Time Target", "%.1f sec", DRIVE_TIME_SECONDS);
        telemetry.addData("Drive Power", "%.2f", DRIVE_POWER);
        telemetry.update();

        waitForStart();

        // Сброс таймера при нажатии START
        timer.reset();

        // Едем прямо, пока идет таймер и автономный режим активен
        while (opModeIsActive() && timer.seconds() < DRIVE_TIME_SECONDS) {
            // Движение прямо вперед
            drivetrain.driveMecanum(DRIVE_POWER, 0.0, 0.0);

            // Обновляем одометрию (если подключен Pinpoint)
            drivetrain.updateOdometry();

            // Вывод телеметрии процесса
            telemetry.addData("Status", "DRIVING STRAIGHT...");
            telemetry.addData("Elapsed Time", "%.2f / %.2f s", timer.seconds(), DRIVE_TIME_SECONDS);
            telemetry.addData("Power", "%.2f", DRIVE_POWER);
            telemetry.addData("Heading", "%.2f deg", drivetrain.getHeadingDegrees());
            telemetry.update();
        }

        // Полная остановка всех моторов
        drivetrain.stopMotors();

        // Финальная телеметрия
        telemetry.addData("Status", "FINISHED! Motors Stopped.");
        telemetry.addData("Total Time", "%.2f s", timer.seconds());
        telemetry.update();
    }
}
