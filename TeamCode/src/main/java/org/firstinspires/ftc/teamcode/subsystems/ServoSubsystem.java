package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ServoSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_servo;
    DistanceSensor m_distanceSensor;
//    CRServo m_servo;

    double m_servoPosition;

    public ServoSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_servo = hardwareMap.get(Servo.class,"SERVO0");
        m_distanceSensor = hardwareMap.get(DistanceSensor.class, "DIST");
        m_servoPosition = 0.0;
    }

    public void setPosition(double position)
    {
        m_servoPosition = position;
    }

    public Command setPositionCommand(double position)
    {
        return new InstantCommand(()-> setPosition(position));
    }
//    private void actuate()
//    {
//        m_launchPosition = (m_launchServo.getPosition() + 1.0) % 2.0;
//    }

//    public Command actuateCommand()
//    {
//        return new InstantCommand(()->{actuate();});
//    }
//    public Command launchPlaneCommand(double launchPosition)
//    {
//        return new InstantCommand(()->{setPosition(launchPosition);});
//    }

    @Override
    public void periodic()
    {
//        m_launchServo.scaleRange(m_launchMinPosition, m_launchMaxPosition);
//        m_launchServo.setPosition(m_launchPosition);
        m_servo.setPosition(m_servoPosition);
        //double distance = m_distanceSensor.getDistance(DistanceUnit.INCH);
//        m_telemetry.addData("Dist", distance);
//        m_telemetry.update();
//        m_telemetry.addData("Servo Position:", m_servo.getPosition());
    }
}
