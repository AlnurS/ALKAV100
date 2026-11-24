package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class Shooting {
    private DcMotorEx shooting1, shooting2;
    private Servo feederServo;
    private VoltageSensor voltageSensor;

    private double targetPower = 0;           // Мощность от 0.0 до 1.0 (доля от максимальной скорости)
    private double targetVelocity = 0;        // Целевая скорость в тиках в секунду (Ticks/sec)
    private double powerCoefficient = RobotConstants.SHOOTING_POWER_COEFFICIENT;

    // Максимальная скорость моторов в тиках в секунду
    public static double DEFAULT_MAX_TICKS_PER_SECOND = RobotConstants.MAX_TICKS_PER_SECOND;

    private double maxVelocityTicksPerSecond = DEFAULT_MAX_TICKS_PER_SECOND;
    private double current1Amps = 0.0;
    private double current2Amps = 0.0;

    private final ElapsedTime spinTimer = new ElapsedTime();
    private boolean spinningUp = false;

    public void init(HardwareMap hardwareMap) {
        try {
            shooting1 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING1_MOTOR_NAME);
            shooting2 = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTING2_MOTOR_NAME);

            // Включаем встроенный аппаратный PIDF-регулятор REV Control Hub (RUN_USING_ENCODER)
            shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Разнонаправленные моторы (инверсия одного из моторов)
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
        setTargetVelocity(-maxVelocityTicksPerSecond * 0.5);
        spinningUp = false;
    }

    public void setPower(double power) {
        this.targetPower = power;
        this.targetVelocity = power * maxVelocityTicksPerSecond * powerCoefficient;
    }

    public void setTargetVelocity(double velocityTicksPerSec) {
        this.targetVelocity = velocityTicksPerSec;
        this.targetPower = maxVelocityTicksPerSecond > 0 ? velocityTicksPerSec / maxVelocityTicksPerSecond : 0;
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
        this.maxVelocityTicksPerSecond = maxVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocityTicksPerSecond;
    }

    public void update() {
        if (shooting1 == null || shooting2 == null) {
            return;
        }

        if (Math.abs(targetVelocity) < 10.0) {
            shooting1.setVelocity(0.0);
            shooting2.setVelocity(0.0);
            return;
        }

        // Встроенный аппаратный PIDF на REV Control Hub точно удерживает заданную скорость
        shooting1.setVelocity(targetVelocity);
        shooting2.setVelocity(targetVelocity);

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
        return shooting1 != null ? shooting1.getVelocity() : 0.0;
    }

    public double getVelocity2() {
        return shooting2 != null ? shooting2.getVelocity() : 0.0;
    }

    public boolean isReady() {
        // Шутер считается готовым, если разгоняется дольше задержки И фактическая скорость моторов досягает >= 90% от целевой
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
        spinningUp = false;
    }

    public int getPos1() { return shooting1 != null ? shooting1.getCurrentPosition() : 0; }
    public int getPos2() { return shooting2 != null ? shooting2.getCurrentPosition() : 0; }
}
