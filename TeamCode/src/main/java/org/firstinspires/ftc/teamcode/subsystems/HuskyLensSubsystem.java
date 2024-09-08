package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HuskyLensSubsystem extends SubsystemBase {
    Telemetry m_telemetry;
    HardwareMap m_hardware;
    HuskyLens m_husky;
    public HuskyLensSubsystem(Telemetry telemetry, HardwareMap hardwareMap){
        m_telemetry = telemetry;
        m_hardware = hardwareMap;
        m_husky = m_hardware.get(HuskyLens.class, "HuskyLens");

        m_husky.initialize();
        m_husky.selectAlgorithm(HuskyLens.Algorithm.NONE);
    }



    public void setAlgorithm(HuskyLens.Algorithm algorithm){
        m_husky.selectAlgorithm(algorithm);
        m_telemetry.addData("Selected Algorithm", algorithm.name());
    }

    public boolean blockDetected(){
        if(m_husky.blocks().length > 0){
            return true;
        }
        return false;
    }

    
}
