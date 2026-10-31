package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
public class RobotConstants {
    // Drivetrain
    public static String LEFT_FRONT = "leftFront";
    public static String RIGHT_FRONT = "rightFront";
    public static String LEFT_BACK = "leftBack";
    public static String RIGHT_BACK = "rightBack";

    public static double NORMAL_SPEED = 1.0;
    public static double SLOW_SPEED = 0.3;

    // Closed-Loop Velocity Control (Максимальная скорость моторов в тиках/сек)
    public static double MAX_TICKS_PER_SECOND = 2500.0;

    // Pinpoint Odometry Configuration
    public static String PINPOINT_NAME = "pinpoint";
    public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /**
     * Смещение одо-подов относительно ЦЕНТРА ВРАЩЕНИЯ робота (в мм):
     * - X_POD_OFFSET_MM: Боковой сдвиг (влево/вправо) X-пода (измеряющего движение ВПЕРЕД/НАЗАД).
     * - Y_POD_OFFSET_MM: Продольный сдвиг (вперед/назад) Y-пода (измеряющего движение ВБОК/СТРЕЙФ).
     */
    public static double X_POD_OFFSET_MM = -100;
    public static double Y_POD_OFFSET_MM = 105;

    // Voltage Compensation
    public static final double NOMINAL_VOLTAGE = 12.2;

    // Lift система лифтов
    public static final String LIFT_MOTOR_LEFT_NAME = "liftLeft";
    public static final String LIFT_MOTOR_RIGHT_NAME = "liftRight";
    public static final int LIFT_POSITION_DOWN = 0;
    public static final int LIFT_POSITION_UP = 2000;
    public static final int LIFT_POSITION_TOLERANCE = 20;
    public static final double LIFT_MAX_POWER = 0.8;
    public static final int LIFT_SOFT_LIMIT_MIN = -50;
    public static final int LIFT_SOFT_LIMIT_MAX = 2100;

    // Intake ЧИТАТЬ С ЛИЦЕВОЙ СТОРОНЫ РОБОТА
    public static final String INTAKE1_MOTOR_NAME = "intake1";
    public static final String INTAKE2_MOTOR_NAME = "intake2";
    public static final double INTAKE_POWER_COLLECT = 1;
    public static final double INTAKE_POWER_REVERSE = -1;
    public static final double INTAKE_SYNC_KP = 0.002;
    public static final double INTAKE_POWER_COEFFICIENT = 1;

    // Shooting (бывший Launcher)
    public static final String SHOOTING1_MOTOR_NAME = "shooting1";
    public static final String SHOOTING2_MOTOR_NAME = "shooting2";
    public static final String FEEDER_SERVO_NAME = "feederServo";
    public static final double SHOOTING_POWER_SPINUP = 1;
    public static final double SHOOTING_POWER_REVERSE = -1;
    public static final double SHOOTING_SYNC_KP = 0.0067;
    public static final double SHOOTING_POWER_COEFFICIENT = 1;
    public static final double SHOOTING_IDLE_POWER = 0.5;
    public static final double SHOOTING_SPINUP_TIME_SEC = 1.0;
    public static final double FEEDER_POSITION_OPEN = 1.0;
    public static final double FEEDER_POSITION_CLOSED = 0.0;

    // Common Servo Constants (Left & Right) intakeSERVO ЧИТАТЬ С ЛИЦЕВОЙ СТОРОНЫ РОБОТА
    public static final String SERVO_LEFT_NAME = "servoLeft";
    public static final String SERVO_RIGHT_NAME = "servoRight";
    public static final double SERVO_POS_1 = 0.0;
    public static final double SERVO_POS_2 = 0.5;
    public static final double SERVO_POS_3 = 1.0;

    // Stopper Servo
    public static final String STOPPER_SERVO_NAME = "stopperServo";
    public static double STOPPER_POS_CLOSED = 70.0;  // 70° в опущенном положении
    public static double STOPPER_POS_OPEN = 104.0;   // 160° в поднятом положении (сильный подъём)

    // ServoRamp (goBILDA 300° Dual Mode Servo на пине 0: "SERVO RAMPA")
    public static final String RAMP_SERVO_NAME = "SERVO RAMPA";
    public static double RAMP_POS_CLOSED = 0.0;  // 0.0 (полностью опущена)
    public static double RAMP_POS_OPEN = 1.0;    // 1.0 (полностью поднята на максимальный угол 300°)
}