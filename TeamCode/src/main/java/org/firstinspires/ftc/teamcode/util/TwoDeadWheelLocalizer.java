package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Constants.DEADWHEEL_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Constants.DEADWHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.Constants.DEADWHEEL_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.PAR_DEADWHEEL_Y_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.PERP_DEADWHEEL_X_OFFSET;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TwoDeadWheelLocalizer {
    double m_distPerTick;
    private boolean m_initialized;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    public TwoDeadWheelLocalizer()
    {
        m_distPerTick = DEADWHEEL_RADIUS * 2 * Math.PI * DEADWHEEL_GEAR_RATIO / DEADWHEEL_TICKS_PER_REV;
        m_initialized = false;
    }

    public Twist2d update(
            int parPos,
            int perpPos,
            double headingRadians
    ) {

        Rotation2d heading = new Rotation2d(headingRadians);

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617

        if (!m_initialized) {
            m_initialized = true;

            lastParPos = parPos;
            lastPerpPos = perpPos;
            lastHeading = heading;

            return new Twist2d(0.0, 0.0, 0.0);
        }

        int parPosDelta = parPos - lastParPos;
        int perpPosDelta = perpPos - lastPerpPos;
        Rotation2d headingDelta = heading.minus(lastHeading);

        Twist2d twist = new Twist2d(
                m_distPerTick * parPosDelta + PAR_DEADWHEEL_Y_OFFSET * headingDelta.getRadians(),  //TODO: double check on '+' in the math
                m_distPerTick * perpPosDelta - PERP_DEADWHEEL_X_OFFSET * headingDelta.getRadians(),
                headingDelta.getRadians()
        );

        lastParPos = parPos;
        lastPerpPos = perpPos;
        lastHeading = heading;

        return twist;
    }

    public void resetHeadingBuffer()
    {
        lastHeading = new Rotation2d(0.0,0.0);
    }

    public void reinitialize()
    {
        m_initialized = false;
    }
}
