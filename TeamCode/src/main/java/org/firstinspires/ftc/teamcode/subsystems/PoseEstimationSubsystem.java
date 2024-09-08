package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.TwoDeadWheelLocalizer;

public class PoseEstimationSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    private TwoDeadWheelLocalizer m_deadWheelLocalizer;

    private Pose2d m_pose2d;
    private double m_headingOffset;
    private IMU m_imu;
    private final DcMotorEx parMotorPort, perpMotorPort;
    private int perpPos, parPos;

    public PoseEstimationSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_imu = m_hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                Constants.LOGO_DIRECTION_IMU,
                Constants.USB_DIRECTION_IMU);
        m_imu.initialize(new IMU.Parameters(orientationOnRobot));
        parMotorPort = m_hardwareMap.get(DcMotorEx.class, "frontRight");
        perpMotorPort = m_hardwareMap.get(DcMotorEx.class, "frontLeft");

        m_deadWheelLocalizer = new TwoDeadWheelLocalizer();

        m_pose2d = new Pose2d(0.0, 0.0, new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        m_headingOffset = 0.0;
    }

    public Command setPoseCommand(Pose2d pose2d)
    {
        return new InstantCommand(()-> setPose(pose2d));
    }

    public void setPose(Pose2d pose2d)
    {
        m_pose2d = pose2d;
        m_deadWheelLocalizer.reinitialize();
    }

    public void setHeadingOffset(double headingOffset)
    {
        m_headingOffset = headingOffset;
    }

    public Pose2d getPose()
    {
        return m_pose2d;
    }

    public double getHeadingOffset()
    {
        return m_headingOffset;
    }

    private void resetHeading()
    {
        m_imu.resetYaw();
        m_deadWheelLocalizer.resetHeadingBuffer();
        m_pose2d = new Pose2d(m_pose2d.getX(),m_pose2d.getY(), new Rotation2d());
        m_headingOffset = 0.0;
    }

    public Command resetHeadingCommand()
    {
        return new InstantCommand(()->{resetHeading();});
    }

    @Override
    public void periodic()
    {
        parPos = -parMotorPort.getCurrentPosition(); //NOTE the direction/sign change here
        perpPos = perpMotorPort.getCurrentPosition();
        double headingRadians = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        Twist2d twistNew = m_deadWheelLocalizer.update(parPos, perpPos, headingRadians);
        m_pose2d = m_pose2d.exp(twistNew);

       // m_telemetry.addData("HEADING", m_pose2d.getHeading());
//        m_telemetry.addData("X", m_pose2d.getX());
//        m_telemetry.addData("Y", m_pose2d.getY());
        //m_telemetry.update();
    }
}
