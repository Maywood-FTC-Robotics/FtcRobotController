package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.util.UnscaledMecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.util.WheelPowers;
import org.firstinspires.ftc.teamcode.util.WheelVoltages;

import java.util.function.Supplier;

@Config
public class MecanumDriveSubsystem extends DriveSubsystem {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;

    UnscaledMecanumDriveKinematics m_mecanumDriveKinematics;
    SimpleMotorFeedforward m_motorFeedforward;
    PhotonLynxVoltageSensor m_voltageSensor;

    private final DcMotorEx m_frontLeft, m_rearLeft, m_rearRight, m_frontRight;
    private Supplier<Pose2d> m_poseSupplier;
    private boolean m_PathFollowingEnabled;
    private Pose2d m_setPointPose2d;
    private double m_setPointVelocity;
    private Rotation2d m_setPointRotation;
    private double m_setPointAngularVelocity;

    private PIDController m_xController, m_yController, m_thetaController;
    private HolonomicDriveController m_holonomicDriveController;

    public static double kp_theta, ki_theta, kd_theta;
    public static double kp_x, ki_x, kd_x;
    public static double kp_y, ki_y, kd_y;

    public MecanumDriveSubsystem(HardwareMap hardwareMap,
                          Telemetry telemetry,
                          Supplier<Pose2d> pose2dSupplier)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_poseSupplier = pose2dSupplier;

        //Create the drivetrain motor instances
        m_frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        m_rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        m_rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        m_frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        //"Zero power" behavior determines how the motor will set at MotorPower = 0 (wheels braking vs wheels floating/coasting)
        m_frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //When you drive forward the wheels should all rotate to move the robot forward.  Adjust if necessary
        m_frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        m_frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Drive kinematics object for converting chassis velocities to wheel velocities
        m_mecanumDriveKinematics = new UnscaledMecanumDriveKinematics(
                Constants.FRONTLEFT_POSITION,
                Constants.FRONTRIGHT_POSITION,
                Constants.BACKLEFT_POSITION,
                Constants.BACKRIGHT_POSITION);

        //Motor feedforward model used to convert wheel velocities to wheel voltages
        m_motorFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

        m_voltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        m_PathFollowingEnabled = false;
        m_setPointPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        m_setPointVelocity = 0.0;
        m_setPointRotation = new Rotation2d(0.0);
        m_setPointAngularVelocity = 0.0;

        //the x,y, theta PID controller constants
        kp_x = 7.0; ki_x = 3.0; kd_x = 0.0;
        kp_y = 7.0; ki_y = 3.0; kd_y = 0.0;
        kp_theta = 6.0; ki_theta = 3.0; kd_theta = 0.0;

        m_xController = new PIDController(kp_x, ki_x, kd_x);
        m_yController = new PIDController(kp_y, ki_y, kd_y);
        m_thetaController = new PIDController(kp_theta, ki_theta, kd_theta);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_holonomicDriveController = new HolonomicDriveController(m_xController,
                m_yController, m_thetaController);
    }

    public void drive(ChassisSpeeds robotChassisSpeeds)
    {
        //Convert robot chassis speeds to individual wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds = m_mecanumDriveKinematics.toWheelSpeeds(robotChassisSpeeds);

        //Convert the wheel speeds to wheel voltages using a motor feedforward model
        WheelVoltages wheelVoltages = computeMotorFeedForward(wheelSpeeds);

        //Normalize the voltages with current voltage to get the % powers for each wheel,
        // then cap/rescale the power to +/-1.0, and set any powers < minLevel to 0.0
//        WheelPowers wheelPowers = wheelVoltages.normalize(12.8);
        WheelPowers wheelPowers = wheelVoltages.normalize(m_voltageSensor.getCachedVoltage());
        wheelPowers.capPower(1.0, 0.01);

        //Set the motor powers
        setMotorPowers(wheelPowers);

//        m_telemetry.addData("VX", robotChassisSpeeds.vxMetersPerSecond);
//        m_telemetry.addData("VY", robotChassisSpeeds.vyMetersPerSecond);
//        m_telemetry.addData("OMEGA", robotChassisSpeeds.omegaRadiansPerSecond);
//        m_telemetry.addData("FL_VOLT", wheelVoltages.frontLeft);
//        m_telemetry.addData("FR_VOLT", wheelVoltages.frontRight);
//        m_telemetry.addData("RL_VOLT", wheelVoltages.rearLeft);
//        m_telemetry.addData("RR_VOLT", wheelVoltages.rearRight);
//        m_telemetry.addData("FL", wheelPowers.frontLeft);
//        m_telemetry.addData("FR", wheelPowers.frontRight);
//        m_telemetry.addData("RL", wheelPowers.rearLeft);
//        m_telemetry.addData("RR", wheelPowers.rearRight);
//        m_telemetry.update();
    }

    private WheelVoltages computeMotorFeedForward(MecanumDriveWheelSpeeds wheelSpeeds)
    {
        WheelVoltages wheelVoltages = new WheelVoltages();
        wheelVoltages.frontLeft = m_motorFeedforward.calculate(wheelSpeeds.frontLeftMetersPerSecond, 0.0);
        wheelVoltages.frontRight = m_motorFeedforward.calculate(wheelSpeeds.frontRightMetersPerSecond, 0.0);
        wheelVoltages.rearLeft = m_motorFeedforward.calculate(wheelSpeeds.rearLeftMetersPerSecond, 0.0);
        wheelVoltages.rearRight = m_motorFeedforward.calculate(wheelSpeeds.rearRightMetersPerSecond, 0.0);
        return wheelVoltages;
    }

    private void setMotorPowers(WheelPowers wheelPowers)
    {
        m_frontLeft.setPower(wheelPowers.frontLeft);
        m_frontRight.setPower(wheelPowers.frontRight);
        m_rearLeft.setPower(wheelPowers.rearLeft);
        m_rearRight.setPower(wheelPowers.rearRight);
    }

    public void updateHeadingSetPoints(Rotation2d desiredRotation, double desiredAngularVelocity)
    {
        m_setPointRotation = desiredRotation;
        m_setPointAngularVelocity = desiredAngularVelocity;
    }
    public void updateSetPoints(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                Rotation2d desiredRotation, double desiredAngularVelocity)
    {
        m_setPointPose2d = desiredTrajectoryPose2d;
        m_setPointVelocity = desiredLinearVelocity;
        m_setPointRotation = desiredRotation;
        m_setPointAngularVelocity = desiredAngularVelocity;
    }

    public Command updateSetPointsCommand(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                          Rotation2d desiredHeading, double desiredAngularVelocity)
    {
        return new InstantCommand(()->{updateSetPoints(desiredTrajectoryPose2d,
                desiredLinearVelocity, desiredHeading, desiredAngularVelocity);});
    }

    public void setPIDFollowing(boolean PIDfollowingEnable)
    {
        m_PathFollowingEnabled = PIDfollowingEnable;
    }

    public Command setPIDFollowingCommand(boolean pathFollowingEnabled)
    {
        return new InstantCommand(()->{setPIDFollowing(pathFollowingEnabled);});
    }
    public void togglePIDFollowing()
    {
        m_PathFollowingEnabled = !m_PathFollowingEnabled;
        m_holonomicDriveController.reset();
    }

    public Command togglePIDFollowingCommand()
    {
        return new InstantCommand(this::togglePIDFollowing);
    }

    private void update() //TODO: figure out and ensure correct units for velocities
    {
        //Update any PID controller coefficients
        m_holonomicDriveController.getXController().setPID(kp_x, ki_x, kd_x);
        m_holonomicDriveController.getYController().setPID(kp_y, ki_y, kd_y);
        m_holonomicDriveController.getThetaController().setPID(kp_theta, ki_theta, kd_theta);

        ChassisSpeeds robotChassisSpeeds = m_holonomicDriveController.calculate(m_poseSupplier.get(),
                m_setPointPose2d, m_setPointVelocity, m_setPointRotation, m_setPointAngularVelocity);

        //Move the robot towards the position/heading/velocity setpoints
        drive(robotChassisSpeeds);
    }

    @Override
    public void periodic()
    {
        if(m_PathFollowingEnabled)
        {
            update();
        }
    }
}