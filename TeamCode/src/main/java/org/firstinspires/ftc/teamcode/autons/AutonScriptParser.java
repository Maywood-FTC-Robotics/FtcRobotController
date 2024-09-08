package org.firstinspires.ftc.teamcode.autons;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.InitializePoseCommand;
import org.firstinspires.ftc.teamcode.commands.ServoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;

public class AutonScriptParser {
    JSONParser m_parser;
    FileReader m_reader;
    DriveSubsystem m_driveSubsystem;
    VisionSubsystem m_visionSubsystem;
    ServoSubsystem m_servoSubsystem;
    PoseEstimationSubsystem m_poseEstimationSubsystem;

    public AutonScriptParser(DriveSubsystem driveSubsystem,
                             PoseEstimationSubsystem poseEstimationSubsystem,
                             VisionSubsystem visionSubsystem,
                             ServoSubsystem servoSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_servoSubsystem = servoSubsystem;
        m_poseEstimationSubsystem = poseEstimationSubsystem;
    }

    @SuppressLint("NewApi")
    public Command read(String inputJsonString) throws IOException {
        Object obj = null;
        m_parser = new JSONParser();

        File file = new File(inputJsonString);
//        DateFormat sdf = new SimpleDateFormat("MMMM dd, yyyy hh:mm a");
//        RobotLog.d(String.format("FILEINFO: canread?: %b\n", file.canRead()));
//        RobotLog.d(String.format("FILEINFO: path: %s", file.getAbsolutePath()));
//        RobotLog.d(String.format("FILEINFO: exists: %b", file.exists()));
//        RobotLog.d(String.format("FILEINFO: modified: %s", sdf.format(file.lastModified())));

        //TODO: trying to figure out why the file appears to be cached or buffered
        // -- it isn't reloading anew when the file modified externally to the process
        // (i.e. when the file is updated and copied to the Robot hub via adb.exe).  These various
        // opening/closing of streams in the code below (along with the reading of the file)
        // seem to fix it, but the root cause is still not understood.

        FileInputStream fileStream = new FileInputStream(file);
        fileStream.close();
        fileStream = new FileInputStream(file);

        int character;
        while ((character = fileStream.read()) != -1) {
//            RobotLog.d("INPUTFILE: %c",(char)character);
        }
        fileStream.close();

        //Here is where I actually read and parse in the json file
        file = new File(inputJsonString);
        m_reader = new FileReader(file);
        //RobotLog.d(String.format("FILEREADER ready?: %b\n", m_reader.ready()));

        try
        {
            obj = m_parser.parse(m_reader);
        }
        catch (ParseException e) {
            e.printStackTrace();
        }

        m_reader.close();
        m_reader = null;
        m_parser = null;

        JSONObject jsonCommandList = (JSONObject)obj;

        Command outCommand = parseJsonCommands(jsonCommandList);
        return outCommand;
    }

    public Command parseJsonCommands(JSONObject jsonCommandList)
    {
        Object key = jsonCommandList.keySet().iterator().next();
        Object firstCommand = jsonCommandList.get(key);

        if(firstCommand instanceof JSONArray) //if the object is a JSONarray (only commandGroups)
        {
            JSONArray firstCommandGroup = (JSONArray)firstCommand;
            CommandGroupBase commandGroup = null;
            switch (key.toString())
            {
                case "SequentialCommandGroup":
                    commandGroup = new SequentialCommandGroup();
                    break;
                case "ParallelCommandGroup":
                    commandGroup = new ParallelCommandGroup();
                    break;
                case "ParallelRaceGroup":
                    commandGroup = new ParallelRaceGroup();
                    break;
            }

            for(Iterator i = firstCommandGroup.iterator(); i.hasNext();)
            {
                JSONObject object = (JSONObject)i.next();
                Command command = parseJsonCommands(object);
                commandGroup.addCommands(command);
            }
            return commandGroup;
        }
        else //If the object is a JSONObject (all other normal commands)
        {
            JSONObject currentObject = (JSONObject)firstCommand;
            switch (key.toString()) {
                case "WaitCommand":
                    Object waitTimeObj = currentObject.get((Object)"WaitTimeMs");
                    int waitTime = ((Long)waitTimeObj).intValue();
                    return new WaitCommand(waitTime);
                case "ConditionalCommand":
                    Object commandIfTrue = currentObject.get((Object)"CommandIfTrue");
                    Command trueCommand = parseJsonCommands((JSONObject)commandIfTrue);

                    Object commandIfFalse = currentObject.get((Object)"CommandIfFalse");
                    Command falseCommand = parseJsonCommands((JSONObject)commandIfFalse);

                    Object conditionLocation = currentObject.get((Object)"Location");
                    int locationCondition = ((Long)conditionLocation).intValue();

                    return new ConditionalCommand(trueCommand, falseCommand, ()-> (m_visionSubsystem.getLocation() == locationCondition));
                case "ServoPositionCommand":
                    Object positionObj = currentObject.get((Object)"Position");
                    double position = ((Double)positionObj).doubleValue();
                    return new ServoPositionCommand(position, m_servoSubsystem);
                case "TrajectoryFollowCommand":
                    Object trajNameObject = currentObject.get((Object)"TrajectoryName");
                    String trajectoryName = trajNameObject.toString();
                    return new TrajectoryFollowCommand(m_driveSubsystem, trajectoryName);
                case "TogglePIDFollowingCommand":
                    return m_driveSubsystem.togglePIDFollowingCommand();
                case "SetPIDFollowingCommand":
                    Object isPIDFollowingEnabled = currentObject.get((Object)"IsEnabled");
                    boolean isEnabled = Boolean.parseBoolean(isPIDFollowingEnabled.toString());
                    return m_driveSubsystem.setPIDFollowingCommand(isEnabled);
                case "SetInitialPoseCommand":
                    Object XObj = currentObject.get((Object)"X");
                    double xPos = ((Double)XObj).doubleValue();
                    Object YObj = currentObject.get((Object)"Y");
                    double yPos = ((Double)YObj).doubleValue();
                    Object headingDegObj = currentObject.get((Object)"HEADINGDEG");
                    double heading = ((Double)headingDegObj).doubleValue() * Math.PI/180.0;
                    Pose2d initialPose = new Pose2d(xPos, yPos, new Rotation2d(heading));
                    return new InitializePoseCommand(initialPose, m_driveSubsystem, m_poseEstimationSubsystem);
                default:
                    return null;
            }
        }
    }

}
