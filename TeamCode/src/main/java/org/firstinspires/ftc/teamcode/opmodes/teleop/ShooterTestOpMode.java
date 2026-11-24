package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooting;

@Config
@TeleOp(name = "Shooter Test OpMode", group = "Test")
public class ShooterTestOpMode extends LinearOpMode {

    // Настраиваемые переменные через FTC Dashboard
    public static double testPower = 1.00;          // Начальная мощность маховиков (100% -> 2000 тиков/сек)
    public static double boostPower = 1.00;         // Максимальная мощность (100%)
    public static boolean autoFeedWhenReady = false; // Авто-подача при достижении скорости

    private final Shooting shooting = new Shooting();
    private final ElapsedTime dpadTimer = new ElapsedTime();

    private boolean isShooterRunning = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastButtonA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Инициализация подсистемы стрельбы
        shooting.init(hardwareMap);

        // Вывод телеметрии как на Driver Hub, так и в браузер FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Шутер готов к тестированию!");
        telemetry.addData("Инструкция", "Нажмите Start (Play) на геймпаде");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. ИЗМЕНЕНИЕ МОЩНОСТИ (D-Pad UP / DOWN) ---
            if (gamepad1.dpad_up && !lastDpadUp && dpadTimer.seconds() > 0.15) {
                testPower = Math.min(1.0, testPower + 0.05);
                dpadTimer.reset();
            }
            if (gamepad1.dpad_down && !lastDpadDown && dpadTimer.seconds() > 0.15) {
                testPower = Math.max(0.0, testPower - 0.05);
                dpadTimer.reset();
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // --- 2. СТАРТ / СТОП МАХОВИКОВ (Кнопка A) ---
            if (gamepad1.a && !lastButtonA) {
                isShooterRunning = !isShooterRunning;
                if (isShooterRunning) {
                    shooting.spinUp();
                } else {
                    shooting.stop();
                }
            }
            lastButtonA = gamepad1.a;

            // --- 3. РЕЖИМЫ РАБОТЫ (R2 - Активация шутера, X - Реверс) ---
            boolean isR2Pressed = gamepad1.right_trigger > 0.1;
            boolean isL2Pressed = gamepad1.left_trigger > 0.1;

            if (gamepad1.x) {
                // Реверс маховиков при кнопке X
                shooting.reverse();
            } else if (isR2Pressed || isShooterRunning) {
                // Включение раскрутки маховиков при зажатии R2 или тумблером A
                double powerToUse = isR2Pressed ? (gamepad1.right_bumper ? boostPower : testPower) : testPower;
                shooting.setPower(powerToUse);
            } else {
                shooting.stop();
            }

            // --- 4. УПРАВЛЕНИЕ ПОДАЧЕЙ (L2 / R1 - FEEDER SERVO) ---
            if (isL2Pressed || gamepad1.right_bumper) {
                // Подача сервоприводом при зажатии L2 или R1
                shooting.feed();
            } else if (autoFeedWhenReady && (isShooterRunning || isR2Pressed) && shooting.isReady()) {
                // Авто-подача при наборе скорости
                shooting.feed();
            } else {
                shooting.stopFeed();
            }

            // Обновление физики и алгоритма компенсации просадки тока
            shooting.update();

            // --- 5. ДИАГНОСТИЧЕСКАЯ ТЕЛЕМЕТРИЯ ---
            telemetry.addData("=== SHOOTER TEST MODE ===", "");
            telemetry.addData("Состояние", isShooterRunning ? "ВКЛЮЧЕН (RUNNING)" : "ВЫКЛЮЧЕН (STOPPED)");
            telemetry.addData("Тестовая мощность (D-Pad)", "%.0f%% (%.2f)", testPower * 100, testPower);
            telemetry.addData("Готовность (Spinup Ready)", shooting.isReady() ? "ГОТОВ (READY)" : "РАЗГОН (SPINNING UP...)");

            telemetry.addData("--- СКОРОСТЬ МОТОРОВ (Ticks/sec) ---", "");
            telemetry.addData("Мотор 1 (Velocity 1)", "%.0f t/s", shooting.getVelocity1());
            telemetry.addData("Мотор 2 (Velocity 2)", "%.0f t/s", shooting.getVelocity2());

            telemetry.addData("--- ТОК И НАГРУЗКА (Amperes) ---", "");
            telemetry.addData("Ток Мотор 1", "%.2f A", shooting.getCurrent1());
            telemetry.addData("Ток Мотор 2", "%.2f A", shooting.getCurrent2());

            telemetry.addData("--- ПИТАНИЕ И ЭНКОДЕРЫ ---", "");
            telemetry.addData("Напряжение АКБ", "%.2f V", shooting.getBatteryVoltage());
            telemetry.addData("Коэффициент вольтажа", "%.3f", shooting.getVoltageCompensationFactor());
            telemetry.addData("Энкодер 1 / 2", "%d / %d", shooting.getPos1(), shooting.getPos2());

            telemetry.update();
        }
    }
}
