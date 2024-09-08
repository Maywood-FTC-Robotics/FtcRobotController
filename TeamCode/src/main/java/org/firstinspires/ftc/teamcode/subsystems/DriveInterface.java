package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public interface DriveInterface {
    public void drive(ChassisSpeeds robotChassisSpeeds);
    public void updateHeadingSetPoints(Rotation2d desiredRotation, double desiredAngularVelocity);
    public void updateSetPoints(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                Rotation2d desiredRotation, double desiredAngularVelocity);
    public Command updateSetPointsCommand(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                          Rotation2d desiredHeading, double desiredAngularVelocity);
    public void setPIDFollowing(boolean PIDfollowingEnable);
    public Command setPIDFollowingCommand(boolean pathFollowingEnabled);
    public void togglePIDFollowing();
    public Command togglePIDFollowingCommand();
}
