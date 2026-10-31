package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * Mecanum Drivetrain с поддержкой GoBilda Pinpoint Odometry & Pedro Pathing.
 * Защищен от сбоев отдельного оборудования (отключенных проводов/несовпадающих имен).
 */
public class Drivetrain {
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;
    private Follower pedroFollower;

    private double speedMultiplier = RobotConstants.NORMAL_SPEED;

    public void init(HardwareMap hardwareMap) {
        try {
            leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_FRONT);
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            leftFront = null;
        }

        try {
            rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_FRONT);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            rightFront = null;
        }

        try {
            leftBack = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_BACK);
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            leftBack = null;
        }

        try {
            rightBack = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_BACK);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            rightBack = null;
        }

        // Датчик напряжения
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Инициализация Pinpoint Odometry
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT_NAME);
            pinpoint.setOffsets(RobotConstants.X_POD_OFFSET_MM, RobotConstants.Y_POD_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            pinpoint.setEncoderDirections(RobotConstants.X_DIRECTION, RobotConstants.Y_DIRECTION);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }

        // Pedro Pathing Follower
        try {
            pedroFollower = new Follower(hardwareMap);
        } catch (Exception e) {
            pedroFollower = null;
        }
    }

    public Follower getPedroFollower() {
        return pedroFollower;
    }

    public void updateOdometry() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {
            }
        }
    }

    public double getBatteryVoltage() {
        if (voltageSensor == null) return RobotConstants.NOMINAL_VOLTAGE;
        try {
            return voltageSensor.getVoltage();
        } catch (Exception e) {
            return RobotConstants.NOMINAL_VOLTAGE;
        }
    }

    public void driveMecanum(double drive, double strafe, double turn) {
        double lf = (drive + strafe + turn) * speedMultiplier;
        double rf = (drive - strafe - turn) * speedMultiplier;
        double lb = (drive - strafe + turn) * speedMultiplier;
        double rb = (drive + strafe - turn) * speedMultiplier;

        double max = Math.max(Math.abs(lf), Math.abs(rf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rb));

        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        try {
            if (leftFront != null) leftFront.setPower(lf);
            if (rightFront != null) rightFront.setPower(rf);
            if (leftBack != null) leftBack.setPower(lb);
            if (rightBack != null) rightBack.setPower(rb);
        } catch (Exception ignored) {
        }
    }

    public void driveMecanumSmooth(double drive, double strafe, double turn) {
        driveMecanum(Math.pow(drive, 3), Math.pow(strafe, 3), Math.pow(turn, 3));
    }

    public void stopMotors() {
        try {
            if (leftFront != null) leftFront.setPower(0);
            if (rightFront != null) rightFront.setPower(0);
            if (leftBack != null) leftBack.setPower(0);
            if (rightBack != null) rightBack.setPower(0);
        } catch (Exception ignored) {
        }
    }

    public Pose2D getPose() {
        if (pinpoint != null) {
            try {
                return pinpoint.getPosition();
            } catch (Exception ignored) {
            }
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    public double getXInches() {
        if (pinpoint != null) {
            try {
                return pinpoint.getPosition().getX(DistanceUnit.INCH);
            } catch (Exception ignored) {
            }
        }
        return 0;
    }

    public double getYInches() {
        if (pinpoint != null) {
            try {
                return pinpoint.getPosition().getY(DistanceUnit.INCH);
            } catch (Exception ignored) {
            }
        }
        return 0;
    }

    public double getHeadingDegrees() {
        if (pinpoint != null) {
            try {
                return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            } catch (Exception ignored) {
            }
        }
        return 0;
    }

    public void resetHeading() {
        if (pinpoint != null) {
            try {
                pinpoint.resetPosAndIMU();
            } catch (Exception ignored) {
            }
        }
    }

    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
    }

    public void setCurrentPose(double xInches, double yInches, double headingDegrees) {
        if (pinpoint != null) {
            try {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
            } catch (Exception ignored) {
            }
        }
    }
}
