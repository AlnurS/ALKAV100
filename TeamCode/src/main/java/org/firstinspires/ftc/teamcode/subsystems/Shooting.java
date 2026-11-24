package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

@Config
public class Shooting {
    private DcMotorEx shooting1, shooting2;
    private Servo feederServo;
    private VoltageSensor voltageSensor;

    private double targetPower = 0;           // Мощность от 0.0 до 1.0
    private double targetVelocity = 0;        // Целевая скорость в тиках в секунду
    private double currentRampVelocity = 0;   // Плавная текущая целевая скорость (Ramp)
    private double lastSetVelocity1 = -99999; 
    private double lastSetVelocity2 = -99999;
    private double powerCoefficient = RobotConstants.SHOOTING_POWER_COEFFICIENT;

    // Целевая рабочая скорость шутера (2000 тиков в секунду)
    public static double MAX_TICKS_PER_SECOND = 2000.0;

    // --- АЭРОДИНАМИЧЕСКИЕ НАСТРОЙКИ (FTC Dashboard) ---
    // Компенсация воздушного сопротивления для Мотора 2 (1.0 = выключено, 1.05 = +5% мощности мотору 2 для преодоления потока воздуха)
    public static double MOTOR2_AIR_BOOST = 1.00;
    // Плавный разгон (чтобы не создавать резкую воздушную стену)
    public static boolean USE_SMOOTH_RAMP = true;
    public static double RAMP_STEP_TICKS = 80.0; // Шаг нарастания скорости за один кадр

    // Калиброванные PIDF параметры для шутера (I = 0)
    public static double PIDF_P = 2.0;
    public static double PIDF_I = 0.0;
    public static double PIDF_D = 0.0;
    public static double PIDF_F = 15.5;

    private double current1Amps = 0.0;
    private double current2Amps = 0.0;

    // Фильтрованные значения скоростей для стабильной телеметрии
    private double filteredVelocity1 = 0.0;
    private double filteredVelocity2 = 0.0;

    private final ElapsedTime spinTimer = new ElapsedTime();
    private boolean spinningUp = false;

    public void init(HardwareMap hardwareMap) {
        try {
            shooting1 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING1_MOTOR_NAME);
            shooting2 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING2_MOTOR_NAME);

            // Режим аппаратного PIDF
            shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Разнонаправленные моторы (инверсия мотора 1)
            shooting1.setDirection(DcMotor.Direction.REVERSE);
            shooting2.setDirection(DcMotor.Direction.FORWARD);

            shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooting2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Применяем коэффициенты
            applyPIDF();
        } catch (Exception e) {
            shooting1 = null;
            shooting2 = null;
        }

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            if (voltageSensor != null && voltageSensor.getVoltage() > 6.0) {
                filteredVoltage = voltageSensor.getVoltage();
            }
        } catch (Exception e) {
            voltageSensor = null;
        }

        try {
            feederServo = hardwareMap.get(Servo.class, RobotConstants.FEEDER_SERVO_NAME);
        } catch (Exception e) {
            feederServo = null;
        }
    }

    public void applyPIDF() {
        if (shooting1 != null && shooting2 != null) {
            PIDFCoefficients pidf = new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
            shooting1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shooting2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }

    public void spinUp() {
        setPower(RobotConstants.SHOOTING_POWER_SPINUP);
        spinningUp = true;
        spinTimer.reset();
    }

    public void idle() {
        setPower(RobotConstants.SHOOTING_IDLE_POWER);
        spinningUp = false;
    }

    public void reverse() {
        setTargetVelocity(-MAX_TICKS_PER_SECOND * 0.5);
        spinningUp = false;
    }

    public void setPower(double power) {
        this.targetPower = power;
        this.targetVelocity = power * MAX_TICKS_PER_SECOND * powerCoefficient;
    }

    public void setTargetVelocity(double velocityTicksPerSec) {
        this.targetVelocity = velocityTicksPerSec;
        this.targetPower = MAX_TICKS_PER_SECOND > 0 ? velocityTicksPerSec / MAX_TICKS_PER_SECOND : 0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetPower() {
        return targetPower;
    }

    public void setPowerCoefficient(double coefficient) {
        this.powerCoefficient = coefficient;
    }

    public double getPowerCoefficient() {
        return powerCoefficient;
    }

    private double filteredVoltage = RobotConstants.NOMINAL_VOLTAGE;

    public double getBatteryVoltage() {
        if (voltageSensor == null) return RobotConstants.NOMINAL_VOLTAGE;
        double currentVoltage = voltageSensor.getVoltage();
        if (currentVoltage < 6.0) {
            return RobotConstants.NOMINAL_VOLTAGE;
        }
        filteredVoltage = 0.05 * currentVoltage + 0.95 * filteredVoltage;
        return filteredVoltage;
    }

    public double getVoltageCompensationFactor() {
        double currentVoltage = getBatteryVoltage();
        return RobotConstants.NOMINAL_VOLTAGE / currentVoltage;
    }

    public void setMaxVelocityTicksPerSecond(double maxVelocity) {
        MAX_TICKS_PER_SECOND = maxVelocity;
    }

    public double getMaxVelocity() {
        return MAX_TICKS_PER_SECOND;
    }

    public void update() {
        if (shooting1 == null || shooting2 == null) {
            return;
        }

        if (Math.abs(targetVelocity) < 10.0) {
            if (lastSetVelocity1 != 0.0 || lastSetVelocity2 != 0.0) {
                shooting1.setVelocity(0.0);
                shooting2.setVelocity(0.0);
                lastSetVelocity1 = 0.0;
                lastSetVelocity2 = 0.0;
            }
            currentRampVelocity = 0.0;
            filteredVelocity1 = 0.0;
            filteredVelocity2 = 0.0;
            return;
        }

        // 1. ПЛАВНЫЙ АЭРОДИНАМИЧЕСКИЙ РАЗГОН (Ramp): не создаем резкий скачок давления воздуха
        if (USE_SMOOTH_RAMP) {
            if (currentRampVelocity < targetVelocity) {
                currentRampVelocity = Math.min(targetVelocity, currentRampVelocity + RAMP_STEP_TICKS);
            } else if (currentRampVelocity > targetVelocity) {
                currentRampVelocity = Math.max(targetVelocity, currentRampVelocity - RAMP_STEP_TICKS);
            }
        } else {
            currentRampVelocity = targetVelocity;
        }

        // 2. АЭРОДИНАМИЧЕСКАЯ КОМПЕНСАЦИЯ ДЛЯ МОТОРА 2 (MOTOR2_AIR_BOOST)
        double v1Target = currentRampVelocity;
        double v2Target = currentRampVelocity * MOTOR2_AIR_BOOST;

        // Отправка команд на моторы только при изменении скоростей
        if (Math.abs(v1Target - lastSetVelocity1) > 1.0) {
            shooting1.setVelocity(v1Target);
            lastSetVelocity1 = v1Target;
        }
        if (Math.abs(v2Target - lastSetVelocity2) > 1.0) {
            shooting2.setVelocity(v2Target);
            lastSetVelocity2 = v2Target;
        }

        // Фильтрация показаний скоростей для стабильного отображения в телеметрии
        double raw1 = shooting1.getVelocity();
        double raw2 = shooting2.getVelocity();
        filteredVelocity1 = 0.25 * raw1 + 0.75 * filteredVelocity1;
        filteredVelocity2 = 0.25 * raw2 + 0.75 * filteredVelocity2;

        // Считывание тока для мониторинга
        try {
            current1Amps = shooting1.getCurrent(CurrentUnit.AMPS);
            current2Amps = shooting2.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            current1Amps = 0.0;
            current2Amps = 0.0;
        }
    }

    public double getCurrent1() {
        return current1Amps;
    }

    public double getCurrent2() {
        return current2Amps;
    }

    public double getVelocity1() {
        return filteredVelocity1 > 0 ? filteredVelocity1 : (shooting1 != null ? shooting1.getVelocity() : 0.0);
    }

    public double getVelocity2() {
        return filteredVelocity2 > 0 ? filteredVelocity2 : (shooting2 != null ? shooting2.getVelocity() : 0.0);
    }

    public boolean isReady() {
        double v1 = Math.abs(getVelocity1());
        double v2 = Math.abs(getVelocity2());
        double target = Math.abs(targetVelocity);
        return spinningUp && target > 100 && (v1 >= target * 0.90) && (v2 >= target * 0.90);
    }

    public void feed() {
        if (feederServo != null) {
            feederServo.setPosition(RobotConstants.FEEDER_POSITION_OPEN);
        }
    }

    public void stopFeed() {
        if (feederServo != null) {
            feederServo.setPosition(RobotConstants.FEEDER_POSITION_CLOSED);
        }
    }

    public void stop() {
        setPower(0);
        targetVelocity = 0;
        currentRampVelocity = 0;
        spinningUp = false;
    }

    public int getPos1() { return shooting1 != null ? shooting1.getCurrentPosition() : 0; }
    public int getPos2() { return shooting2 != null ? shooting2.getCurrentPosition() : 0; }
}
