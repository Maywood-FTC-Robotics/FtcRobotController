package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.util.ArrayList;
import java.util.List;

public class PathParameters {
    public static class Point {
        public double x;
        public double y;
        public double pathHeadingDeg;

        public Point(double x, double y, double pathHeadingDeg) {
            this.x = x;
            this.y = y;
            this.pathHeadingDeg = pathHeadingDeg;
        }
    }

    public Pose2d startPoint;
    public List<Translation2d> wayPoints;
    public Pose2d endPoint;
    public double maxVelocityMetersPerSec;
    public double maxAccelerationMetersPerSecondSq;
    public boolean isReversed;
    public double startVelocity;
    public double endVelocity;
    public double maxAngularVelocityDegPerSec;
    public double maxAngularAccelerationDegPerSecondSq;
    public double startAngleDeg;
    public double endAngleDeg;

    public PathParameters(JSONObject jsonObject) {
        JSONObject startPointJson = (JSONObject) jsonObject.get("StartPoint");
        this.startPoint = new Pose2d(
                (double) startPointJson.get("X"),
                (double) startPointJson.get("Y"),
                new Rotation2d((double)startPointJson.get("pathHeadingDeg"))
        );

        JSONObject wayPointsJson = (JSONObject) jsonObject.get("WayPoints");
        JSONArray wayPointsX = (JSONArray) wayPointsJson.get("X");
        JSONArray wayPointsY = (JSONArray) wayPointsJson.get("Y");

        this.wayPoints = new ArrayList<>();
        for (int i = 0; i < wayPointsX.size(); i++) {
            this.wayPoints.add(new Translation2d(
                    (double) wayPointsX.get(i),
                    (double) wayPointsY.get(i)
            ));
        }

        JSONObject endPointJson = (JSONObject) jsonObject.get("EndPoint");
        this.endPoint = new Pose2d(
                (double) endPointJson.get("X"),
                (double) endPointJson.get("Y"),
                new Rotation2d((double) endPointJson.get("pathHeadingDeg"))
        );

        this.maxVelocityMetersPerSec = (double) jsonObject.get("MaxVelocityMetersPerSec");
        this.maxAccelerationMetersPerSecondSq = (double) jsonObject.get("MaxAccelerationMetersPerSecondSq");
        this.isReversed = Boolean.parseBoolean((String) jsonObject.get("IsReversed"));
        this.startVelocity = (double) jsonObject.get("StartVelocity");
        this.endVelocity = (double) jsonObject.get("EndVelocity");
        this.maxAngularVelocityDegPerSec = (double) jsonObject.get("MaxAngularVelocityDegPerSec");
        this.maxAngularAccelerationDegPerSecondSq = (double) jsonObject.get("MaxAngularAccelerationDegPerSecondSq");
        this.startAngleDeg = (double) jsonObject.get("StartAngleDeg");
        this.endAngleDeg = (double) jsonObject.get("EndAngleDeg");
    }
}
