package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Config
public class TeleopDriveCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Pose2d> m_pose2dSupplier;
    private final DoubleSupplier m_leftYSupplier;
    private final DoubleSupplier m_leftXSupplier;
    private final DoubleSupplier m_rightXSupplier;
    private final DoubleSupplier m_rightTriggerSupplier;
    private final DoubleSupplier m_headingOffsetSupplier;
    private boolean m_isFieldCentric;
    private final BooleanSupplier m_isPIDThetaEnabledSupplier;

    private PIDController m_thetaController;
    public static double m_right_stick_deadzone = 0.05;
    public static double m_kp = 0.4;
    private double m_thetaSetPoint;

    public TeleopDriveCommand(DriveSubsystem drive,
                        Supplier<Pose2d> pose2dSupplier,
                        DoubleSupplier leftYSupplier,
                        DoubleSupplier leftXSupplier,
                        DoubleSupplier rightXSupplier,
                        DoubleSupplier rightTriggerSupplier,
                        DoubleSupplier headingOffsetSupplier,
                        BooleanSupplier isPIDThetaEnabledSupplier,
                        boolean isFieldCentric) {

        m_driveSubsystem = drive;
        m_pose2dSupplier = pose2dSupplier;
        m_leftXSupplier = leftXSupplier;
        m_leftYSupplier = leftYSupplier;
        m_rightXSupplier = rightXSupplier;
        m_rightTriggerSupplier = rightTriggerSupplier;
        m_headingOffsetSupplier = headingOffsetSupplier;
        m_isFieldCentric = isFieldCentric;
        m_isPIDThetaEnabledSupplier = isPIDThetaEnabledSupplier;
        m_thetaController = new PIDController(m_kp, 0.0, 0.0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        double heading = m_pose2dSupplier.get().getHeading();
        double headingOffset = m_headingOffsetSupplier.getAsDouble();
        m_thetaSetPoint = heading + headingOffset;
    }

    @Override
    public void execute() {
        double throttle = m_rightTriggerSupplier.getAsDouble();
        double leftX = m_leftXSupplier.getAsDouble();
        double leftY = m_leftYSupplier.getAsDouble();
        double rightX = m_rightXSupplier.getAsDouble();
        double heading = m_pose2dSupplier.get().getHeading();
        double headingOffset = m_headingOffsetSupplier.getAsDouble();

        //"Turbo" trigger functionality
        double throttleSlope = 1 - Constants.THROTTLEMINLEVEL;
        double throttleScale = throttleSlope * throttle + Constants.THROTTLEMINLEVEL;

        //Compute field-centric speeds from controller inputs. Note: the axes of the game
        // controller analog stick axes will not match the axes of the FTC field.
        double vel_x = leftY * throttleScale * Constants.MAX_VELOCITY;
        double vel_y = leftX * throttleScale * Constants.MAX_VELOCITY;
        double omega = rightX * throttleScale * Constants.MAX_ANGULAR_VELOCITY;

        //When the theta PID theta controller engaged, override the input from the right stick
        if((Math.abs(rightX) < m_right_stick_deadzone) && m_isPIDThetaEnabledSupplier.getAsBoolean())
        {
            m_thetaController.setP(m_kp);
            rightX = m_thetaController.calculate(heading + headingOffset, m_thetaSetPoint);
            omega = rightX * Constants.MAX_ANGULAR_VELOCITY;
        }
        else //For normal driving, the setpoint is always assigned to the current measured heading
        {
            m_thetaSetPoint = heading + headingOffset;
        }

        //Convert Field-centric speeds to robot-centric speeds
        ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel_x, vel_y, omega,
                new Rotation2d(m_isFieldCentric ? (heading + headingOffset): 0.0));

        m_driveSubsystem.drive(robotChassisSpeeds);
    }
}

