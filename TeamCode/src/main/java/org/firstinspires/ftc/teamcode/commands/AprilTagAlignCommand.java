package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Constants.INCHES_TO_METERS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.HolonomicDriveController;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
public class AprilTagAlignCommand extends CommandBase {

    DriveSubsystem m_driveSubsystem;
    VisionSubsystem m_visionSubsystem;

    PIDController m_thetaController, m_xController, m_yController;
    public static double kp_theta = -5.0;
    public static double kp_x = -5.0;
    public static double kp_y = 5.0;

    public AprilTagAlignCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;

        m_thetaController = new PIDController(kp_theta, 0.0, 0.0);
        m_xController = new PIDController(kp_x, 0.0,0.0);
        m_yController = new PIDController(kp_y, 0.0,0.0);

        addRequirements(m_driveSubsystem, m_visionSubsystem);
    }

    @Override
    public void execute()
    {
        m_thetaController.setP(kp_theta);
        m_xController.setP(kp_x);
        m_yController.setP(kp_y);
        AprilTagPoseFtc ftcPose = m_visionSubsystem.getFtcPose();
        double desired_angle = 0.0; //Radians
        double desired_range = 12 * INCHES_TO_METERS;
        double desired_y = 0.0 * INCHES_TO_METERS;
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        if (ftcPose != null) {
            double omega_d = m_thetaController.calculate(ftcPose.yaw * Math.PI/180.0, desired_angle);
            double velx_d = m_xController.calculate(INCHES_TO_METERS * ftcPose.range, desired_range);
            double vely_d = m_yController.calculate(INCHES_TO_METERS * ftcPose.x, desired_y);
            robotSpeeds = new ChassisSpeeds(velx_d, vely_d, omega_d);
        }
        m_driveSubsystem.drive(robotSpeeds);
    }

}
