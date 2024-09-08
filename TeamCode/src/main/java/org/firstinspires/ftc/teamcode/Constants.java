package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class Constants
{
    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    public enum DrivetrainType {
        SWERVE,
        MECANUM
    }

    public static double INCHES_TO_METERS = 0.0254;

    //Dead Wheel Odometry Constants
//    public static double DEADWHEEL_RADIUS = 0.944882; //inches
    public static double DEADWHEEL_RADIUS = 0.024; //m
    public static double DEADWHEEL_GEAR_RATIO = 1.0;
    public static double DEADWHEEL_TICKS_PER_REV = 2000;
    public static double PERP_DEADWHEEL_X_OFFSET = 0.0;  //TODO: determine better way of estimating this
    public static double PAR_DEADWHEEL_Y_OFFSET = -158.6e-3;   //TODO: determine better way of estimating this

    //Driving constants
    public static final double THROTTLEMINLEVEL = 0.2;
    public static boolean FIELD_CENTRIC_DRIVING = true;
    public static boolean ROBOT_CENTRIC_DRIVING = false;
    public static double RED_HEADING_OFFSET = -Math.PI/2.0;
    public static double BLUE_HEADING_OFFSET = Math.PI/2.0;

    //SET THE DRIVETRAIN TYPE
    private static final DrivetrainType DRIVETRAIN_TYPE = DrivetrainType.SWERVE;

    //Drivetrain constants
    public static double TRACK_WIDTH;
    public static double WHEEL_BASE;
    public static double WHEEL_RADIUS;
    public static double MAX_RPM;
    public static double MAX_VELOCITY;
    public static double MAX_ANGULAR_VELOCITY;
    public static double MAX_VOLTAGE = 13.5;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION_IMU;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION_IMU;

    //Assign the constants different values based on drivetrain type
    static {
        switch (DRIVETRAIN_TYPE) {
            case SWERVE:
                TRACK_WIDTH = 0.2667; //distance between both front wheels (in m) (or both rear wheels)
                WHEEL_BASE = 0.2794; //distance between the front and rear wheel on one side (in m)
                WHEEL_RADIUS = 0.057; //TODO: measure radius of a single drive wheel (m)
                MAX_RPM = 1000; //TODO: update the max RPM of drive motors
                MAX_VELOCITY = 2.8; // max robot linear velocity (m/sec)
                MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH / 2.0 + WHEEL_BASE / 2.0); //TODO: check derivation of angular vel. from previous constants
                LOGO_DIRECTION_IMU = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
                USB_DIRECTION_IMU = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
                break;
            case MECANUM:
                TRACK_WIDTH = 317.2e-3; //distance between both front wheels (in m) (or both rear wheels)
                WHEEL_BASE = 240e-3; //distance between the front and rear wheel on one side (in m)
                WHEEL_RADIUS = 0.048; //Radius of a single drive wheel (m)
                MAX_RPM = 223; //max RPM of drive motors
                MAX_VELOCITY = MAX_RPM * 2.0 * Math.PI * WHEEL_RADIUS / 60.0;// max robot linear velocity (m/sec)
                MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH / 2.0 + WHEEL_BASE / 2.0); //TODO: check derivation of angular vel. from previous constants
                LOGO_DIRECTION_IMU = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
                USB_DIRECTION_IMU = RevHubOrientationOnRobot.UsbFacingDirection.UP;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + DRIVETRAIN_TYPE);
        }
    }

    //Position of the wheel modules (for both drivetrains) and center of rotation.
    // Move these into the case/switch statement above if newer drivetrains deviate from this
    public static Translation2d BACKLEFT_POSITION = new Translation2d(-Constants.WHEEL_BASE/2.0, Constants.TRACK_WIDTH/2.0);
    public static Translation2d FRONTLEFT_POSITION = new Translation2d(Constants.WHEEL_BASE/2.0, Constants.TRACK_WIDTH/2.0);
    public static Translation2d FRONTRIGHT_POSITION = new Translation2d(Constants.WHEEL_BASE/2.0 , -Constants.TRACK_WIDTH/2.0);
    public static Translation2d BACKRIGHT_POSITION = new Translation2d(-Constants.WHEEL_BASE/2.0, -Constants.TRACK_WIDTH/2.0);
    public static Translation2d CENTER_OF_ROTATION = new Translation2d(0,0);

    //Drive motor (Static, Velocity, & Acceleration) feedforward constants
    // (NOTE: these need to be determined from tuning scripts)
    public static double kS = 0.01;
    public static double kV = MAX_VOLTAGE/MAX_VELOCITY;
    public static double kA = 0.001;


}


