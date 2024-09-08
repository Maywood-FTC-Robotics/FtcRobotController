package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
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
import org.opencv.core.Mat;

import java.io.IOException;

public class TrajectoryFollowCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final String m_trajectoryFile;
    private ElapsedTime m_timer;
    private PathParameters m_pathParameters;
    private AutonPathParser m_autonPathParser;
    private Trajectory m_trajectory;
    private TrapezoidProfile m_thetaMotionProfile;

    public TrajectoryFollowCommand(DriveSubsystem driveSubsystem, String trajectoryName)
        {
        m_timer = new ElapsedTime();
        m_driveSubsystem = driveSubsystem;
        m_trajectoryFile = "/sdcard/AutonPaths/" + trajectoryName + ".json";
        m_autonPathParser = new AutonPathParser();

        try {
            m_pathParameters = m_autonPathParser.read(m_trajectoryFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                m_pathParameters.maxVelocityMetersPerSec,
                m_pathParameters.maxAccelerationMetersPerSecondSq);
        trajectoryConfig.setReversed(m_pathParameters.isReversed);
        trajectoryConfig.setStartVelocity(m_pathParameters.startVelocity);
        trajectoryConfig.setEndVelocity(m_pathParameters.endVelocity);

        m_trajectory = TrajectoryGenerator.generateTrajectory(m_pathParameters.startPoint,
                m_pathParameters.wayPoints,
                m_pathParameters.endPoint,
                trajectoryConfig);

        TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
                m_pathParameters.maxAngularVelocityDegPerSec,
                m_pathParameters.maxAngularAccelerationDegPerSecondSq);
        TrapezoidProfile.State startGoal = new TrapezoidProfile.State(m_pathParameters.startAngleDeg, 0.0);
        TrapezoidProfile.State endGoal = new TrapezoidProfile.State(m_pathParameters.endAngleDeg, 0.0);

        m_thetaMotionProfile = new TrapezoidProfile(thetaConstraints, endGoal,startGoal);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        m_timer.reset();
        double currentTime = m_timer.time();
        Trajectory.State initialPose = m_trajectory.sample(currentTime);
        TrapezoidProfile.State initialHeading = m_thetaMotionProfile.calculate(currentTime);

//        RobotLog.d("TIMETRAJECTORY; t:%f x:%f velx:%f\n",currentTime,
//                initialPose.poseMeters.getX(), initialPose.velocityMetersPerSecond);
//        RobotLog.d("ANGLETRAJECTORY; t:%f ang:%f angvel:%f\n",currentTime,
//                initialHeading.position, initialHeading.velocity);

        m_driveSubsystem.updateSetPoints(initialPose.poseMeters,
                initialPose.velocityMetersPerSecond,
                new Rotation2d(initialHeading.position * Math.PI/180.0),
                initialHeading.velocity * Math.PI/180.0);
    }

    @Override
    public void execute()
    {
        double currentTime = m_timer.time();
        Trajectory.State currentPose = m_trajectory.sample(currentTime);
        TrapezoidProfile.State currentHeading = m_thetaMotionProfile.calculate(currentTime);

        m_driveSubsystem.updateSetPoints(currentPose.poseMeters,
                currentPose.velocityMetersPerSecond,
                new Rotation2d(currentHeading.position * Math.PI/180.0),
                currentHeading.velocity * Math.PI/180.0);

//        RobotLog.d("TIMETRAJECTORY; t:%f x:%f velx:%f\n",currentTime,
//                currentPose.poseMeters.getX(), currentPose.velocityMetersPerSecond);
//        RobotLog.d("ANGLETRAJECTORY; t:%f ang:%f angvel:%f\n",currentTime,
//                currentHeading.position, currentHeading.velocity);
    }

    @Override
    public void end(boolean interrupted)
    {
        //TODO: check if the endHeading needs to be calculated with the thetaMotionProfile's final time of the trajectory's final time...
        Trajectory.State endPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds());
        TrapezoidProfile.State endHeading = m_thetaMotionProfile.calculate(m_thetaMotionProfile.totalTime());

        m_driveSubsystem.updateSetPoints(endPose.poseMeters,
                endPose.velocityMetersPerSecond,
                new Rotation2d(endHeading.position * Math.PI/180.0),
                endHeading.velocity * Math.PI/180.0);

//        RobotLog.d("TIMETRAJECTORY; t:%f x:%f velx:%f\n",m_timer.time(),
//                endPose.poseMeters.getX(), endPose.velocityMetersPerSecond);
//        RobotLog.d("ANGLETRAJECTORY; t:%f ang:%f angvel:%f\n",m_thetaMotionProfile.totalTime(),
//                endHeading.position, endHeading.velocity);

        if(interrupted)
        {
            //m_driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished()
    {
        if(m_timer.time() > m_trajectory.getTotalTimeSeconds())
            return true;
        else
            return false;
        //return Thread.currentThread().isInterrupted() || //!m_driveSubsystem.isBusy();
    }
}
