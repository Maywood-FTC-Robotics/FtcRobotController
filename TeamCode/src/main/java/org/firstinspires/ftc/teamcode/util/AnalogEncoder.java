package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;


public class AnalogEncoder{
    AnalogInput m_voltage;

    Boolean IS_INVERTED;
    public AnalogEncoder(AnalogInput voltageSensor){
        m_voltage = voltageSensor;
    }

    public void setIS_INVERTED(Boolean IS_INVERTED) {
        this.IS_INVERTED = IS_INVERTED;
    }

    public double getAngle(){
        return (IS_INVERTED? (-m_voltage.getVoltage() / 3.3) * 360 : (m_voltage.getVoltage() / 3.3) * 360);

    }

}
