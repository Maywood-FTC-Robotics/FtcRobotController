package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autons.AutonScriptParser;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.IOException;
import java.util.List;

@Config
public class RobotContainer extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    //Subsystems
    //VisionSubsystem m_visionSS;
    PoseEstimationSubsystem m_poseEstSS;
    DriveSubsystem m_driveSS;
    //ServoSubsystem m_servoSS;

    //Auton script parser
    AutonScriptParser m_autonScriptParser;
    String m_autoName;

    public RobotContainer(HardwareMap hardwareMap,
                      Telemetry telemetry,
                      Gamepad gamePad1,
                      Gamepad gamePad2,
                      Constants.OpModeType opModeType) throws IOException {
        this(hardwareMap,
                telemetry,
                gamePad1,
                gamePad2,
                opModeType,
                "");
    }

    public RobotContainer(HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                     Gamepad gamePad2,
                     Constants.OpModeType opModeType,
                     String autoName) throws IOException {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_gamePad2 = new GamepadEx(gamePad2);
        m_telemetry = telemetry;

        //TODO: CHECK IF THIS CLASS IS REALLY NEEDED?
        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems (order of init. may matter for optimal updates)
        //m_visionSS = new VisionSubsystem(m_hardwareMap, m_telemetry);
        m_poseEstSS = new PoseEstimationSubsystem(m_hardwareMap, m_telemetry);
        //m_driveSS = new SwerveDriveSubsystem(m_hardwareMap, m_telemetry, m_poseEstSS::getPose);
        m_driveSS = new MecanumDriveSubsystem(m_hardwareMap, m_telemetry, m_poseEstSS::getPose);

        //   m_servoSS = new ServoSubsystem(m_hardwareMap, m_telemetry);

        //The file name of the requested auton
        m_autoName = autoName;

        //Set the hardware to do bulk reads to speed up loop time (see GM0 for details).
        //This may need adjustment if faster reads needed or if reads need to be multiple times/loop
        List<LynxModule> allHubs = m_hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        switch(opModeType) {
            case TELEOP:
                teleopInit();
                m_telemetry.addData("Initialized","TeleOp");
                break;
            case AUTO:
                //autonInit();
                m_telemetry.addData("Initialized Auto", m_autoName);
                break;
            default:
                m_telemetry.addData("Unknown Type", opModeType);
                break;
        }
        m_telemetry.update();
    }

    private void teleopInit()
    {
        //Driver Operator Mappings
        m_driveSS.setDefaultCommand(new TeleopDriveCommand(m_driveSS,
                m_poseEstSS::getPose,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                m_poseEstSS::getHeadingOffset,
                ()->false, //TODO: replace with a state variable
                Constants.FIELD_CENTRIC_DRIVING));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new ServoPositionCommand(1.0, m_servoSS))
//                .whenReleased(new ServoPositionCommand(0.0, m_servoSS));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.BACK)
//                        .whenPressed(m_driveSS.togglePIDFollowingCommand());
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(m_poseEstSS.resetHeadingCommand());
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                        .whenHeld(new AprilTagAlignCommand(m_driveSS, m_visionSS));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new RotateProfileCommand(m_driveSS, m_poseEstSS.getPose().getHeading() * 180.0/Math.PI, 90.0));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(m_driveSS.updateSetPointsCommand(
//                        new Pose2d(0.25, 0.0, Math.toRadians(90)),
//                        new PoseVelocity2d(new Vector2d(0.0,0.0),0.0)
//                ));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(m_driveSS.updateSetPointsCommand(
//                        new Pose2d(0.0, 0.0, new Rotation2d(0,0))
//                ));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(m_poseEstSS.resetHeadingCommand());

        //Operator Control Mappings
    }

//    private void autonInit() throws IOException {
//        String autonPath = "/sdcard/AutonScripts/" + m_autoName + ".json";
//
//        //Initialize the auton script parser (inject all necessary subsystems) and read auton script
//        m_autonScriptParser = new AutonScriptParser(m_driveSS, m_poseEstSS, m_visionSS, m_servoSS);
//        Command autoCommand = m_autonScriptParser.read(autonPath);
//
//        //Schedule and run the parsed auton command
//        CommandScheduler.getInstance().schedule(autoCommand);
//    }

//    public void disableVision() //TODO might need to remove these next two methods
//    {
//        m_visionSubsystem.disablePipeline();
//    }

    public void restoreFromPoseStorage(Pose2d inputPose, double headingOffset)
    {
        m_poseEstSS.setPose(inputPose);
        m_poseEstSS.setHeadingOffset(headingOffset);
    }
    public Pose2d getRobotPose()
    {
        return m_poseEstSS.getPose();
    }
}

