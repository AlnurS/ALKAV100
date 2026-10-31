package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class Shooting {
    private DcMotorEx shooting1, shooting2;
    private Servo feederServo;
    private VoltageSensor voltageSensor;

    private double targetPower = 0;
    private double powerCoefficient = RobotConstants.SHOOTING_POWER_COEFFICIENT;

    // Порог детекции токовой нагрузки (в Амперах)
    public static double CURRENT_LOAD_THRESHOLD_AMPS = 3.5; // Выше 3.5 А считается просадкой под нагрузкой
    private double current1Amps = 0.0;
    private double current2Amps = 0.0;

    // Максимальная скорость моторов в тиках в секунду
    public static final double DEFAULT_MAX_TICKS_PER_SECOND = 2800.0;

    private final ElapsedTime spinTimer = new ElapsedTime();
    private boolean spinningUp = false;
    private double activeBoostFactor = 1.0;

    public void init(HardwareMap hardwareMap) {
        try {
            shooting1 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING1_MOTOR_NAME);
            shooting2 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING2_MOTOR_NAME);

            // Прямой режим RUN_WITHOUT_ENCODER для мгновенной подачи мощности
            shooting1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooting2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Разнонаправленные моторы (инвертированное направление стрельбы)
            shooting1.setDirection(DcMotor.Direction.REVERSE);
            shooting2.setDirection(DcMotor.Direction.FORWARD);

            shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooting2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        setPower(RobotConstants.SHOOTING_POWER_REVERSE);
        spinningUp = false;
    }

    public void setPower(double power) {
        this.targetPower = power;
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

    private double maxVelocityTicksPerSecond = DEFAULT_MAX_TICKS_PER_SECOND;

    public void setMaxVelocityTicksPerSecond(double maxVelocity) {
        this.maxVelocityTicksPerSecond = maxVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocityTicksPerSecond;
    }

    public void update() {
        if (shooting1 == null || shooting2 == null) {
            return;
        }

        if (Math.abs(targetPower) < 0.05) {
            shooting1.setPower(0.0);
            shooting2.setPower(0.0);
            activeBoostFactor = 1.0;
            return;
        }

        // Считываем токовую нагрузку моторов в Амперах (Amps) через встроенные датчики REV Control Hub
        try {
            current1Amps = shooting1.getCurrent(CurrentUnit.AMPS);
            current2Amps = shooting2.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            current1Amps = 0.0;
            current2Amps = 0.0;
        }

        // ДЕТЕКЦИЯ ПРОСАДКИ ПО ТОКУ (Current Draw Sensing):
        // Когда элемент сдавливает маховики, токовая потребность моторов мгновенно скачет вверх.
        double targetBoost = 1.0;
        if (current1Amps > CURRENT_LOAD_THRESHOLD_AMPS || current2Amps > CURRENT_LOAD_THRESHOLD_AMPS) {
            targetBoost = 1.25; // Автоматическое форсирование +25% мощности под токовой нагрузкой
        }

        // Плавный асимметричный гистерезис подхвата мощности
        if (activeBoostFactor < targetBoost) {
            activeBoostFactor = Math.min(targetBoost, activeBoostFactor + 0.08); // Быстрый подхват при скачке тока
        } else if (activeBoostFactor > targetBoost) {
            activeBoostFactor = Math.max(targetBoost, activeBoostFactor - 0.01);  // Мягкий спад после вылета
        }

        // Вычисляем итоговую мощность с компенсацией напряжения аккумулятора и токового форсажа
        double compensatedPower = targetPower * powerCoefficient * getVoltageCompensationFactor() * activeBoostFactor;
        compensatedPower = Range.clip(compensatedPower, -1.0, 1.0);

        shooting1.setPower(compensatedPower);
        shooting2.setPower(compensatedPower);
    }

    public double getCurrent1() {
        return current1Amps;
    }

    public double getCurrent2() {
        return current2Amps;
    }

    public double getVelocity1() {
        return shooting1 != null ? shooting1.getVelocity() : 0.0;
    }

    public double getVelocity2() {
        return shooting2 != null ? shooting2.getVelocity() : 0.0;
    }

    public boolean isReady() {
        return spinningUp && spinTimer.seconds() > RobotConstants.SHOOTING_SPINUP_TIME_SEC;
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
        spinningUp = false;
    }

    public int getPos1() { return shooting1 != null ? shooting1.getCurrentPosition() : 0; }
    public int getPos2() { return shooting2 != null ? shooting2.getCurrentPosition() : 0; }
}
