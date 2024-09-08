package org.firstinspires.ftc.teamcode.util;

public class WheelVoltages {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

    public WheelVoltages()
    {
        this(0.0,0.0,0.0,0.0);
    }
    public WheelVoltages(double fl, double fr, double rl, double rr)
    {
        frontLeft = fl;
        frontRight = fr;
        rearLeft = rl;
        rearRight = rr;
    }

    public WheelPowers normalize(double maxVoltage)
    {
        return new WheelPowers(frontLeft/maxVoltage,
                frontRight/maxVoltage,
                rearLeft/maxVoltage,
                rearRight/maxVoltage);
    }
}
