package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class Intake {
    private DcMotorEx intake1, intake2;
    private VoltageSensor voltageSensor;
    private double targetPower = 0;
    private double powerCoefficient = RobotConstants.INTAKE_POWER_COEFFICIENT;

    // Максимальная скорость моторов интейка в тиках в секунду (для 1150 RPM / 435 RPM моторов)
    public static final double DEFAULT_MAX_TICKS_PER_SECOND = 2000.0;
    private double maxVelocityTicksPerSecond = DEFAULT_MAX_TICKS_PER_SECOND;

    public void init(HardwareMap hardwareMap) {
        try {
            intake1 = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE1_MOTOR_NAME);
            intake2 = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE2_MOTOR_NAME);

            // Сброс энкодеров
            intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Режим RUN_USING_ENCODER включает аппаратный PIDF регулятор скорости на REV Control Hub (1000 Гц)
            intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Разные направления моторов для интейка (один вперед, второй в реверс)
            intake1.setDirection(DcMotor.Direction.FORWARD);
            intake2.setDirection(DcMotor.Direction.REVERSE);

            intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intake1 = null;
            intake2 = null;
        }

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            if (voltageSensor != null && voltageSensor.getVoltage() > 6.0) {
                filteredVoltage = voltageSensor.getVoltage();
            }
        } catch (Exception e) {
            voltageSensor = null;
        }
    }

    public void collect() {
        setPower(RobotConstants.INTAKE_POWER_COLLECT);
    }

    public void reverse() {
        setPower(RobotConstants.INTAKE_POWER_REVERSE);
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

    public double getCompensatedPower() {
        if (Math.abs(targetPower) < 0.05) {
            return 0.0;
        }
        double compensatedPower = targetPower * powerCoefficient * getVoltageCompensationFactor();
        return Range.clip(compensatedPower, -1.0, 1.0);
    }

    public void setMaxVelocityTicksPerSecond(double maxVelocity) {
        this.maxVelocityTicksPerSecond = maxVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocityTicksPerSecond;
    }

    public void update() {
        if (intake1 == null || intake2 == null) {
            return;
        }

        if (Math.abs(targetPower) < 0.05) {
            intake1.setVelocity(0);
            intake2.setVelocity(0);
            return;
        }

        // Вычисляем целевую скорость в тиках в секунду (ticks / sec)
        double maxVel = getMaxVelocity();
        double targetVelocity = targetPower * powerCoefficient * maxVel;

        // Передаем скорость в аппаратный PIDF-контроллер REV Hub (работает на частоте 1000 Гц)
        // Точно так же, как в подсистеме Shooting
        intake1.setVelocity(targetVelocity);
        intake2.setVelocity(targetVelocity);
    }

    public void stop() {
        setPower(0);
    }

    public double getVelocity1() { return intake1 != null ? intake1.getVelocity() : 0.0; }
    public double getVelocity2() { return intake2 != null ? intake2.getVelocity() : 0.0; }
    public int getPos1() { return intake1 != null ? intake1.getCurrentPosition() : 0; }
    public int getPos2() { return intake2 != null ? intake2.getCurrentPosition() : 0; }
}
