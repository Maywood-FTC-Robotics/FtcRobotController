package org.firstinspires.ftc.teamcode.autons;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;

public class AutonPathParser {
    JSONParser m_parser;
    FileReader m_reader;

    public AutonPathParser(){};

    public PathParameters read(String inputJsonString) throws IOException {
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

        return new PathParameters((JSONObject)obj);
    }
}
