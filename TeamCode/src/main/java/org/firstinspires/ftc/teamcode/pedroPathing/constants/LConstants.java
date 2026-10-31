package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Localization constants for Pedro Pathing using GoBilda Pinpoint driver.
 */
@Config
public class LConstants {
    public static String pinpointName = RobotConstants.PINPOINT_NAME;
    public static double xOffsetMM = RobotConstants.X_POD_OFFSET_MM;
    public static double yOffsetMM = RobotConstants.Y_POD_OFFSET_MM;
    public static DistanceUnit distanceUnit = DistanceUnit.MM;
    public static GoBildaPinpointDriver.EncoderDirection xDirection = RobotConstants.X_DIRECTION;
    public static GoBildaPinpointDriver.EncoderDirection yDirection = RobotConstants.Y_DIRECTION;
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
}
