package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;

public class ServoPositionCommand extends CommandBase {

    ServoSubsystem m_servoSubystem;
    double m_position;

    public ServoPositionCommand(
            double position,
            ServoSubsystem servoSubsystem)
    {
        m_position = position;
        m_servoSubystem = servoSubsystem;
        addRequirements(m_servoSubystem);
    }

    @Override
    public void initialize() {
        m_servoSubystem.setPosition(m_position);
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
