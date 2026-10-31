package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ServoRamp;
import org.firstinspires.ftc.teamcode.subsystems.Shooting;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name = "FRC_SRIMMAGE", group = "TeleOp")
public class FRC_SRIMMAGE extends LinearOpMode {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Stopper stopper = new Stopper();
    private final ServoRamp servoRamp = new ServoRamp();
    private final Shooting shooting = new Shooting();

    // Состояние ServoRamp (Поднята / Опущена)
    private boolean isRampUp = false;
    private boolean lastTriangleState = false;

    // Один мотор забора (Intake) на максимальной скорости с плавным рампингом
    private DcMotorEx singleIntakeMotor;
    private double currentIntakePower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Инициализация подсистем
        try { drivetrain.init(hardwareMap); } catch (Exception ignored) {}
        try { stopper.init(hardwareMap); } catch (Exception ignored) {}
        try { servoRamp.init(hardwareMap); } catch (Exception ignored) {}
        try { shooting.init(hardwareMap); } catch (Exception ignored) {}
        try { shooting.setPowerCoefficient(1.0); } catch (Exception ignored) {}

        // Инициализация ОДНОГО мотора забора
        try {
            singleIntakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE1_MOTOR_NAME);
            singleIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            singleIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Прямое управление максимальной мощностью
            singleIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
            singleIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            singleIntakeMotor = null;
        }

        telemetry.addData("Status", "FRC_SRIMMAGE Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Одометрия Pinpoint
            try {
                drivetrain.updateOdometry();
            } catch (Exception ignored) {}

            // 2. Сброс угла кнопкой Options или Dpad Up
            if (gamepad1.options || gamepad1.dpad_up) {
                try {
                    drivetrain.resetHeading();
                } catch (Exception ignored) {}
            }

            // 3. ДВИЖЕНИЕ РОБОТА (Mecanum)
            try {
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;
                drivetrain.driveMecanum(drive, strafe, turn);
            } catch (Exception ignored) {}

            // 4. УПРАВЛЕНИЕ INTAKE МОТОРОМ (L1 / R1)
            try {
                double targetIntakePower = 0.0;
                if (gamepad1.left_bumper) {
                    targetIntakePower = 0.7;  // 80% мощность
                } else if (gamepad1.right_bumper) {
                    targetIntakePower = -0.7; // -80% реверс
                }

                double rampStep = 0.08;
                if (currentIntakePower < targetIntakePower) {
                    currentIntakePower = Math.min(targetIntakePower, currentIntakePower + rampStep);
                } else if (currentIntakePower > targetIntakePower) {
                    currentIntakePower = Math.max(targetIntakePower, currentIntakePower - rampStep);
                }

                if (singleIntakeMotor != null) {
                    singleIntakeMotor.setPower(currentIntakePower);
                }
            } catch (Exception ignored) {}

            // 5. УПРАВЛЕНИЕ STOPPER (Кнопка Circle / B -> 160° сильный подъём, Иначе -> 70°)
            try {
                if (gamepad1.circle || gamepad1.b) {
                    stopper.setDegrees(RobotConstants.STOPPER_POS_OPEN); // 160° (поднято выше)
                } else {
                    stopper.setDegrees(RobotConstants.STOPPER_POS_CLOSED); // 70° (опущено)
                }
            } catch (Exception ignored) {}

            // 6. УПРАВЛЕНИЕ SERVO RAMP (Кнопка Triangle / Y -> Тогл Поднята / Опущена)
            try {
                boolean currentTriangleState = gamepad1.triangle || gamepad1.y;
                if (currentTriangleState && !lastTriangleState) {
                    isRampUp = !isRampUp; // Переключение при нажатии на треугольник
                }
                lastTriangleState = currentTriangleState;

                if (isRampUp) {
                    servoRamp.setUp();   // Полностью поднята (позиция 1.0)
                } else {
                    servoRamp.setDown(); // Полностью опущена (позиция 0.0)
                }
            } catch (Exception ignored) {}

            // 7. УПРАВЛЕНИЕ СТРЕЛЬБОЙ (Shooting)
            try {
                boolean shooterBoost = gamepad1.right_trigger > 0.1;
                shooting.setPower(0.55);
                shooting.update();
            } catch (Exception ignored) {}

            // 8. ТЕЛЕМЕТРИЯ НА DRIVER HUB
            try {
                telemetry.addData("--- SINGLE INTAKE (SMOOTH MAX) ---", "");
                telemetry.addData("Intake Power", "%.2f (Smooth 100%%)", currentIntakePower);

                telemetry.addData("--- STOPPER SERVO ---", "");
                telemetry.addData("Stopper Degree", "%.1f°", stopper.getDegrees());
                telemetry.addData("Stopper Position", "%.2f", stopper.getPosition());

                telemetry.addData("--- SERVO RAMP ---", "");
                if (servoRamp.isAttached()) {
                    telemetry.addData("Ramp Hardware", "CONNECTED (%s)", servoRamp.getAttachedName());
                    telemetry.addData("Ramp State", "%s (Pos: %.2f)", isRampUp ? "UP (ПОДНЯТА)" : "DOWN (ОПУЩЕНА)", servoRamp.getPosition());
                } else {
                    telemetry.addData("Ramp Hardware", "НЕ НАЙДЕН! Проверьте имя в Driver Hub!");
                }

                telemetry.addData("--- SHOOTING (DcMotorEx PIDF) ---", "");
                telemetry.addData("Motor 1 Velocity", "%.0f t/s", shooting.getVelocity1());
                telemetry.addData("Motor 2 Velocity", "%.0f t/s", shooting.getVelocity2());

                telemetry.addData("--- ODOMETRY (Pinpoint) ---", "");
                telemetry.addData("X (Inches)", "%.2f in", drivetrain.getXInches());
                telemetry.addData("Y (Inches)", "%.2f in", drivetrain.getYInches());
                telemetry.addData("Heading", "%.2f deg", drivetrain.getHeadingDegrees());

                telemetry.addData("--- SYSTEM ---", "");
                telemetry.addData("Battery Voltage", "%.2f V", drivetrain.getBatteryVoltage());
                telemetry.update();
            } catch (Exception ignored) {}
        }
    }
}
