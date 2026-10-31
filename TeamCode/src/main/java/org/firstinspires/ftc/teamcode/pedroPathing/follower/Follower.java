package org.firstinspires.ftc.teamcode.pedroPathing.follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathgen.Point;

/**
 * Standard Pedro Pathing Follower class with Minimum Turning Torque & 4-Wheel Mecanum Rotation.
 */
public class Follower {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;

    private Pose currentPose = new Pose(0, 0, 0);
    private Pose targetPose = new Pose(0, 0, 0);

    private PathChain currentPathChain;
    private int currentPathIndex = 0;
    private boolean isFollowing = false;
    private boolean holdPositionAtEnd = true;

    private double maxPower = 1.0;
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // PD Controller Memory
    private double lastXError = 0.0;
    private double lastYError = 0.0;
    private double lastHeadingError = 0.0;
    private boolean firstLoop = true;

    public Follower(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, FConstants.leftFrontMotorName);
        rightFront = hardwareMap.get(DcMotor.class, FConstants.rightFrontMotorName);
        leftBack = hardwareMap.get(DcMotor.class, FConstants.leftRearMotorName);
        rightBack = hardwareMap.get(DcMotor.class, FConstants.rightRearMotorName);

        leftFront.setDirection(FConstants.leftFrontMotorDirection);
        leftBack.setDirection(FConstants.leftRearMotorDirection);
        rightFront.setDirection(FConstants.rightFrontMotorDirection);
        rightBack.setDirection(FConstants.rightRearMotorDirection);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Pinpoint Odometry
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, LConstants.pinpointName);
            pinpoint.setOffsets(LConstants.xOffsetMM, LConstants.yOffsetMM, LConstants.distanceUnit);
            pinpoint.setEncoderResolution(LConstants.encoderResolution);
            pinpoint.setEncoderDirections(LConstants.xDirection, LConstants.yDirection);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }
    }

    public boolean isPinpointConnected() {
        return pinpoint != null;
    }

    public void setStartingPose(Pose pose) {
        this.currentPose = pose;
        this.targetPose = pose;
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
        }
        resetPIDState();
    }

    public void setPose(Pose pose) {
        setStartingPose(pose);
    }

    public Pose getPose() {
        return currentPose;
    }

    public void followPath(Path path, boolean holdPosition) {
        followPath(new PathChain(path), holdPosition);
    }

    public void followPath(Path path) {
        followPath(path, true);
    }

    public void followPath(PathChain pathChain, boolean holdPosition) {
        this.currentPathChain = pathChain;
        this.currentPathIndex = 0;
        this.isFollowing = true;
        this.holdPositionAtEnd = holdPosition;
        this.pathTimer.reset();
        resetPIDState();

        if (pathChain.size() > 0) {
            Path firstPath = pathChain.get(0);
            Point endPt = firstPath.getEndPoint();
            this.targetPose = new Pose(endPt.getX(), endPt.getY(), firstPath.getEndHeading());
        }
    }

    public void followPath(PathChain pathChain) {
        followPath(pathChain, true);
    }

    public void breakFollowing() {
        isFollowing = false;
        currentPathChain = null;
        stopMotors();
    }

    public boolean isBusy() {
        return isFollowing;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastXError = 0.0;
        lastYError = 0.0;
        lastHeadingError = 0.0;
        firstLoop = true;
        loopTimer.reset();
    }

    public void update() {
        // Safety check: if pinpoint is disconnected or null, stop motors immediately
        if (pinpoint == null) {
            stopMotors();
            isFollowing = false;
            return;
        }

        updatePose();

        if (!isFollowing) {
            return;
        }

        if (currentPathChain == null || currentPathIndex >= currentPathChain.size()) {
            isFollowing = false;
            stopMotors();
            return;
        }

        Path currentPath = currentPathChain.get(currentPathIndex);
        Point startPoint = currentPath.getStartPoint();
        Point endPoint = currentPath.getEndPoint();

        double totalDist = startPoint.distanceFrom(endPoint);
        double distFromStart = startPoint.distanceFrom(currentPose.getPoint());

        // Алгоритм с упреждением (Lookahead Distance): целевая точка всегда находится впереди на пару дюймов
        double lookaheadInches = totalDist > 0.01 ? 6.0 : 0.0;
        double t = totalDist > 0.01 ? Range.clip((distFromStart + lookaheadInches) / totalDist, 0.0, 1.0) : 1.0;

        // Динамическая упреждающая целевая точка P(t) на кривой Безье или прямой
        Point targetPoint = currentPath.getPoint(t);

        double targetHeading;
        if (currentPath.getInterpolationMode() == Path.HeadingInterpolation.LINEAR) {
            double startH = currentPath.getStartHeading();
            double endH = currentPath.getEndHeading();
            double hDiff = AngleUnit.normalizeRadians(endH - startH);
            targetHeading = AngleUnit.normalizeRadians(startH + t * hDiff);
        } else if (currentPath.getInterpolationMode() == Path.HeadingInterpolation.TANGENT) {
            // Угол касательной вдоль кривой Безье
            Point p1 = currentPath.getPoint(Math.max(0.0, t - 0.02));
            Point p2 = currentPath.getPoint(Math.min(1.0, t + 0.02));
            double dx = p2.getX() - p1.getX();
            double dy = p2.getY() - p1.getY();
            targetHeading = Math.hypot(dx, dy) > 0.001 ? Math.atan2(dy, dx) : currentPath.getEndHeading();
        } else {
            targetHeading = currentPath.getEndHeading();
        }

        targetPose = new Pose(targetPoint.getX(), targetPoint.getY(), targetHeading);

        // Расстояние до реального конца отрезка endPoint (для отслеживания завершения)
        double endXError = endPoint.getX() - currentPose.getX();
        double endYError = endPoint.getY() - currentPose.getY();
        double distanceError = Math.hypot(endXError, endYError);

        // Ошибка до динамической целевой точки с упреждением (Lookahead Target Point)
        double driveXError = targetPoint.getX() - currentPose.getX();
        double driveYError = targetPoint.getY() - currentPose.getY();

        double headingError = AngleUnit.normalizeRadians(targetHeading - currentPose.getHeading());

        // Check path completion
        if (distanceError < FConstants.pathEndTranslationalConstraint &&
            Math.abs(headingError) < FConstants.pathEndHeadingConstraint) {

            currentPathIndex++;
            pathTimer.reset();

            if (currentPathIndex >= currentPathChain.size()) {
                isFollowing = false;
                if (!holdPositionAtEnd) {
                    stopMotors();
                }
                return;
            } else {
                // Move to next path segment
                Path nextPath = currentPathChain.get(currentPathIndex);
                Point nextPoint = nextPath.getEndPoint();
                targetPose = new Pose(nextPoint.getX(), nextPoint.getY(), nextPath.getEndHeading());
            }
        }

        if (pathTimer.seconds() > FConstants.pathEndTimeout) {
            // Timeout safety check
            currentPathIndex++;
            pathTimer.reset();
            if (currentPathIndex >= currentPathChain.size()) {
                isFollowing = false;
                stopMotors();
                return;
            }
        }

        // Calculate time delta for D-term
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double xErrorD = 0.0;
        double yErrorD = 0.0;
        double headingErrorD = 0.0;

        if (!firstLoop && dt > 0.001 && dt < 0.2) {
            xErrorD = (driveXError - lastXError) / dt;
            yErrorD = (driveYError - lastYError) / dt;
            headingErrorD = (headingError - lastHeadingError) / dt;
        } else {
            firstLoop = false;
        }

        lastXError = driveXError;
        lastYError = driveYError;
        lastHeadingError = headingError;

        // PD calculations
        double drivePower = Range.clip(driveXError * FConstants.transP + xErrorD * FConstants.transD, -1.0, 1.0);
        double strafePower = Range.clip(driveYError * FConstants.transP + yErrorD * FConstants.transD, -1.0, 1.0);
        double turnPower = Range.clip(-(headingError * FConstants.headingP + headingErrorD * FConstants.headingD), -1.0, 1.0);

        // Стандартная зона нечувствительности (Deadband) на основе системных ограничений FConstants:
        // Если робот находится внутри допуска по координатам, линейная мощность сбрасывается в 0
        if (distanceError < FConstants.pathEndTranslationalConstraint) {
            drivePower = 0.0;
            strafePower = 0.0;
        }

        // Если робот находится внутри допуска по углу, разворот сбрасывается в 0
        if (Math.abs(headingError) < FConstants.pathEndHeadingConstraint) {
            turnPower = 0.0;
        } else {
            double minTurnPower = 0.25; // Минимальная мощность для уверенного преодоления трения
            if (Math.abs(turnPower) < minTurnPower && Math.abs(turnPower) > 0.01) {
                turnPower = Math.signum(turnPower) * minTurnPower;
            }
        }

        // Convert field centric to robot centric
        double botHeading = currentPose.getHeading();
        double rotX = drivePower * Math.cos(botHeading) + strafePower * Math.sin(botHeading);
        double rotY = -drivePower * Math.sin(botHeading) + strafePower * Math.cos(botHeading);

        driveMecanum(rotX, rotY, turnPower);
    }

    private void updatePose() {
        if (pinpoint != null) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            currentPose = new Pose(
                p.getX(DistanceUnit.INCH),
                p.getY(DistanceUnit.INCH),
                p.getHeading(AngleUnit.RADIANS)
            );
        }
    }

    public void driveMecanum(double drive, double strafe, double turn) {
        double leftFrontPower = (drive + strafe + turn) * maxPower;
        double rightFrontPower = (drive - strafe - turn) * maxPower;
        double leftBackPower = (drive - strafe + turn) * maxPower;
        double rightBackPower = (drive + strafe - turn) * maxPower;

        double voltageFactor = getVoltageCompensationFactor();
        leftFrontPower *= voltageFactor;
        rightFrontPower *= voltageFactor;
        leftBackPower *= voltageFactor;
        rightBackPower *= voltageFactor;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private double getVoltageCompensationFactor() {
        if (voltageSensor == null) return 1.0;
        double v = voltageSensor.getVoltage();
        if (v < 6.0) return 1.0;
        return RobotConstants.NOMINAL_VOLTAGE / v;
    }
}
