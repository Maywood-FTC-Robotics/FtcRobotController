package org.firstinspires.ftc.teamcode.util;

import java.util.stream.DoubleStream;

public class WheelPowers {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

    public WheelPowers(double fl, double fr, double rl, double rr)
    {
        frontLeft = fl;
        frontRight = fr;
        rearLeft = rl;
        rearRight = rr;
    }

    public void capPower(double maxAchievablePower, double minAchievablePower)
    {
        //Get the largest wheel abs(power)
        double maxPower = DoubleStream.of(Math.abs(frontLeft),
                        Math.abs(frontRight),
                        Math.abs(rearLeft),
                        Math.abs(rearRight))
                .max().getAsDouble();

        //Scale the powers back down to something that can be run on the robot
        if(maxPower > maxAchievablePower)
        {
            frontLeft /= maxPower;
            frontRight /= maxPower;
            rearRight /= maxPower;
            rearLeft /= maxPower;
        }

        //If the powers are below the min level, set the power to 0
        frontLeft = (Math.abs(frontLeft) > minAchievablePower) ? frontLeft : 0.0;
        frontRight = (Math.abs(frontRight) > minAchievablePower) ? frontRight : 0.0;
        rearRight = (Math.abs(rearRight) > minAchievablePower) ? rearRight : 0.0;
        rearLeft = (Math.abs(rearLeft) > minAchievablePower) ? rearLeft : 0.0;
    }
}
