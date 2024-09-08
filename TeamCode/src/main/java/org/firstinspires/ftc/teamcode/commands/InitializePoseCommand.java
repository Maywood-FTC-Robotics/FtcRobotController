package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.autons.AutonPathParser;
import org.firstinspires.ftc.teamcode.autons.PathParameters;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;

import java.io.IOException;

public class InitializePoseCommand extends CommandBase {

    DriveSubsystem m_driveSubsystem;
    PoseEstimationSubsystem m_poseEstimationSubsystem;
    Pose2d m_initialPose;

    public InitializePoseCommand(
            Pose2d initialPose,
            DriveSubsystem driveSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem)
    {
        m_initialPose = initialPose;
        m_driveSubsystem = driveSubsystem;
        m_poseEstimationSubsystem = poseEstimationSubsystem;
        addRequirements(m_driveSubsystem, m_poseEstimationSubsystem);
    }

    public InitializePoseCommand(
            String trajectoryName,
            DriveSubsystem driveSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem)
    {
        String trajectoryFile = "/sdcard/AutonPaths/" + trajectoryName + ".json";
        AutonPathParser autonPathParser = new AutonPathParser();
        PathParameters pathParameters;

        try {
            pathParameters = autonPathParser.read(trajectoryFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        m_initialPose = new Pose2d(pathParameters.startPoint.getX(),
                pathParameters.startPoint.getY(),
                new Rotation2d(pathParameters.startAngleDeg * Math.PI/180.0));
        m_driveSubsystem = driveSubsystem;
        m_poseEstimationSubsystem = poseEstimationSubsystem;
        addRequirements(m_driveSubsystem, m_poseEstimationSubsystem);
    }

    @Override
    public void initialize() {
        m_poseEstimationSubsystem.setPose(m_initialPose);
        m_driveSubsystem.updateSetPoints(m_initialPose, 0.0,
                m_initialPose.getRotation(), 0.0);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
