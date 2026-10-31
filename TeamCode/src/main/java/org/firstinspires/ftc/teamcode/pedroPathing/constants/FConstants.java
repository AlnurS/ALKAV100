package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Follower PID and physical robot constants for Pedro Pathing.
 */
@Config
public class FConstants {
    public static String leftFrontMotorName = RobotConstants.LEFT_FRONT;
    public static String rightFrontMotorName = RobotConstants.RIGHT_FRONT;
    public static String leftRearMotorName = RobotConstants.LEFT_BACK;
    public static String rightRearMotorName = RobotConstants.RIGHT_BACK;

    // Все моторы в FORWARD кроме rightRear
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

    // Translational PID coefficients (X, Y motion)
    public static double transP = 0.22;
    public static double transI = 0.0;
    public static double transD = 0.023;

    // Heading PID coefficients (Rotation) - Увеличен headingP для быстрого и уверенного разворота
    public static double headingP = 1.3;
    public static double headingI = 0.0;
    public static double headingD = 0.1;

    // Physical Robot Parameters (Масса и физика робота)
    public static double mass = 5.3; // Масса робота в кг (используется для расчета инерции и центростремительной силы)
    public static double centripetalScaling = 0.0005; // Коэффициент компенсации сноса на дугах

    // Path End Constraints (Допуски точного прибытия в целевую точку)
    public static double pathEndTimeout = 3.0; // Seconds
    public static double pathEndTranslationalConstraint = 0.4; // 0.4 дюйма (~1 см) точный допуск по X/Y
    public static double pathEndHeadingConstraint = Math.toRadians(1.5); // 1.5° точный допуск по углу
}
