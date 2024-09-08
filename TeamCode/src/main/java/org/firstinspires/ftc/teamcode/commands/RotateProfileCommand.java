package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autons.AutonPathParser;
import org.firstinspires.ftc.teamcode.autons.PathParameters;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.io.IOException;

public class RotateProfileCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private ElapsedTime m_timer;

    private TrapezoidProfile m_thetaMotionProfile;

    public RotateProfileCommand(DriveSubsystem driveSubsystem, double startAngleDeg, double endAngleDeg)
    {
        m_timer = new ElapsedTime();
        m_driveSubsystem = driveSubsystem;

        TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
                180.0,
                180.0);
        TrapezoidProfile.State startGoal = new TrapezoidProfile.State(startAngleDeg, 0.0);
        TrapezoidProfile.State endGoal = new TrapezoidProfile.State(endAngleDeg, 0.0);

        m_thetaMotionProfile = new TrapezoidProfile(thetaConstraints, endGoal,startGoal);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        m_timer.reset();
        double currentTime = m_timer.time();
        TrapezoidProfile.State initialHeading = m_thetaMotionProfile.calculate(currentTime);

        m_driveSubsystem.updateHeadingSetPoints(
                new Rotation2d(initialHeading.position * Math.PI/180.0),
                initialHeading.velocity * Math.PI/180.0);
    }

    @Override
    public void execute()
    {
        double currentTime = m_timer.time();
        TrapezoidProfile.State currentHeading = m_thetaMotionProfile.calculate(currentTime);

        m_driveSubsystem.updateHeadingSetPoints(
                new Rotation2d(currentHeading.position * Math.PI/180.0),
                currentHeading.velocity * Math.PI/180.0);
    }

    @Override
    public void end(boolean interrupted)
    {
        //TODO: check if the endHeading needs to be calculated with the thetaMotionProfile's final time of the trajectory's final time...
        TrapezoidProfile.State endHeading = m_thetaMotionProfile.calculate(m_thetaMotionProfile.totalTime());

        m_driveSubsystem.updateHeadingSetPoints(
                new Rotation2d(endHeading.position * Math.PI/180.0),
                endHeading.velocity * Math.PI/180.0);

        if(interrupted)
        {
            //m_driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished()
    {
        if(m_timer.time() > m_thetaMotionProfile.totalTime())
            return true;
        else
            return false;
        //return Thread.currentThread().isInterrupted() || //!m_driveSubsystem.isBusy();
    }
}
