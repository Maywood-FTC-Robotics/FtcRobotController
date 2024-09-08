package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics.normalizeWheelSpeeds;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.util.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.util.SwerveModule;

import java.util.function.Supplier;

@Config
public class SwerveDriveSubsystem extends DriveSubsystem {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;

    SimpleMotorFeedforward m_motorFeedforward;
    PhotonLynxVoltageSensor m_voltageSensor;

    //private final DcMotorEx m_frontLeft, m_rearLeft, m_rearRight, m_frontRight;
    private Supplier<Pose2d> m_poseSupplier;
    private boolean m_PathFollowingEnabled;
    private Pose2d m_setPointPose2d;
    private double m_setPointVelocity;
    private Rotation2d m_setPointRotation;
    private double m_setPointAngularVelocity;

    private PIDController m_xController, m_yController, m_thetaController;
    private HolonomicDriveController m_holonomicDriveController;

    public static double kp_theta, ki_theta, kd_theta;
    public static double kp_x, ki_x, kd_x;
    public static double kp_y, ki_y, kd_y;

    public static double FLOffset = 110.22;
    public static double FROffset = -97.138;
    public static double BLOffest = 60 ;
    public static double BROffest = -165.81;

    DcMotorEx m_frontLeftMotor, m_rearLeftMotor, m_rearRightMotor, m_frontRightMotor;
    CRServo m_frontLeftServo, m_rearLeftServo, m_rearRightServo, m_frontRightServo;
    SwerveModule m_frontLeftModule, m_backLeftModule, m_backRightModule, m_frontRightModule;
    AnalogEncoder m_frontLeftEncoder, m_frontRightEncoder, m_rearRightEncoder, m_rearLeftEncoder;


    private final SwerveDriveKinematics m_swerveDriveKinematics;
    public SwerveDriveSubsystem(HardwareMap hardwareMap,
                                Telemetry telemetry,
                                Supplier<Pose2d> pose2dSupplier)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_poseSupplier = pose2dSupplier;

        //Create Swerve Modules
        m_frontLeftMotor = m_hardwareMap.get(DcMotorEx.class, "frontLeft");
        m_rearLeftMotor = m_hardwareMap.get(DcMotorEx.class, "rearLeft");
        m_rearRightMotor = m_hardwareMap.get(DcMotorEx.class, "rearRight");
        m_frontRightMotor = m_hardwareMap.get(DcMotorEx.class, "frontRight");

        m_frontLeftServo = m_hardwareMap.get(CRServo.class, "frontLeftS");
        m_rearLeftServo = m_hardwareMap.get(CRServo.class, "rearLeftS");
        m_rearRightServo = m_hardwareMap.get(CRServo.class, "rearRightS");
        m_frontRightServo = m_hardwareMap.get(CRServo.class, "frontRightS");

        m_frontLeftEncoder = new AnalogEncoder(m_hardwareMap.get(AnalogInput.class, "frontLeftE"));
        m_rearLeftEncoder = new AnalogEncoder(m_hardwareMap.get(AnalogInput.class, "rearLeftE"));
        m_rearRightEncoder = new AnalogEncoder(m_hardwareMap.get(AnalogInput.class, "rearRightE"));
        m_frontRightEncoder = new AnalogEncoder(m_hardwareMap.get(AnalogInput.class, "frontRightE"));

        m_backLeftModule = new SwerveModule(m_rearLeftMotor, m_rearLeftServo, m_rearLeftEncoder , BLOffest);
        m_backRightModule = new SwerveModule(m_rearRightMotor, m_rearRightServo, m_rearRightEncoder , BROffest);
        m_frontLeftModule = new SwerveModule(m_frontLeftMotor, m_frontLeftServo, m_frontLeftEncoder , FLOffset);
        m_frontRightModule = new SwerveModule(m_frontRightMotor, m_frontRightServo, m_frontRightEncoder , FROffset);

        //Drive kinematics object for converting chassis velocities to wheel velocities
        m_swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.BACKLEFT_POSITION,
                Constants.BACKRIGHT_POSITION,
                Constants.FRONTRIGHT_POSITION,
                Constants.FRONTLEFT_POSITION);

        //Motor feedforward model used to convert wheel velocities to wheel voltages
        m_motorFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

        m_voltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        m_PathFollowingEnabled = false;
        m_setPointPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        m_setPointVelocity = 0.0;
        m_setPointRotation = new Rotation2d(0.0);
        m_setPointAngularVelocity = 0.0;

        //the x,y, theta PID controller constants
        kp_x = 7.0; ki_x = 3.0; kd_x = 0.0;
        kp_y = 7.0; ki_y = 3.0; kd_y = 0.0;
        kp_theta = 6.0; ki_theta = 3.0; kd_theta = 0.0;

        m_xController = new PIDController(kp_x, ki_x, kd_x);
        m_yController = new PIDController(kp_y, ki_y, kd_y);
        m_thetaController = new PIDController(kp_theta, ki_theta, kd_theta);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_holonomicDriveController = new HolonomicDriveController(m_xController,
                m_yController, m_thetaController);
    }

    public void drive(ChassisSpeeds robotChassisSpeeds)
    {
        //Compute desired module states from input robot speeds
        SwerveModuleState[] moduleStates = m_swerveDriveKinematics.toSwerveModuleStates(
                robotChassisSpeeds,
                Constants.CENTER_OF_ROTATION);

        normalizeWheelSpeeds(moduleStates, Constants.MAX_VELOCITY);
        m_backLeftModule.setState(moduleStates[0]); // Back left module state
        m_backRightModule.setState(moduleStates[1]); // Back right module state
        m_frontRightModule.setState(moduleStates[2]); // Front right module state
        m_frontLeftModule.setState(moduleStates[3]); // Front left module state

    }

    public void updateHeadingSetPoints(Rotation2d desiredRotation, double desiredAngularVelocity)
    {
        m_setPointRotation = desiredRotation;
        m_setPointAngularVelocity = desiredAngularVelocity;
    }
    public void updateSetPoints(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                Rotation2d desiredRotation, double desiredAngularVelocity)
    {
        m_setPointPose2d = desiredTrajectoryPose2d;
        m_setPointVelocity = desiredLinearVelocity;
        m_setPointRotation = desiredRotation;
        m_setPointAngularVelocity = desiredAngularVelocity;
    }

    public Command updateSetPointsCommand(Pose2d desiredTrajectoryPose2d, double desiredLinearVelocity,
                                          Rotation2d desiredHeading, double desiredAngularVelocity)
    {
        return new InstantCommand(()->{updateSetPoints(desiredTrajectoryPose2d,
                desiredLinearVelocity, desiredHeading, desiredAngularVelocity);});
    }

    public void setPIDFollowing(boolean PIDfollowingEnable)
    {
        m_PathFollowingEnabled = PIDfollowingEnable;
    }

    public Command setPIDFollowingCommand(boolean pathFollowingEnabled)
    {
        return new InstantCommand(()->{setPIDFollowing(pathFollowingEnabled);});
    }
    public void togglePIDFollowing()
    {
        m_PathFollowingEnabled = !m_PathFollowingEnabled;
        m_holonomicDriveController.reset();
    }

    public Command togglePIDFollowingCommand()
    {
        return new InstantCommand(this::togglePIDFollowing);
    }

    private void update() //TODO: figure out and ensure correct units for velocities
    {
        //Update any PID controller coefficients
        m_holonomicDriveController.getXController().setPID(kp_x, ki_x, kd_x);
        m_holonomicDriveController.getYController().setPID(kp_y, ki_y, kd_y);
        m_holonomicDriveController.getThetaController().setPID(kp_theta, ki_theta, kd_theta);

        ChassisSpeeds robotChassisSpeeds = m_holonomicDriveController.calculate(m_poseSupplier.get(),
                m_setPointPose2d, m_setPointVelocity, m_setPointRotation, m_setPointAngularVelocity);

        //Move the robot towards the position/heading/velocity setpoints
        drive(robotChassisSpeeds);
    }

    @Override
    public void periodic()
    {
        m_telemetry.addData("servo Position", m_frontLeftModule.getServoAngle());
        m_telemetry.addData("servo target", m_frontLeftModule.getCurrentTarget());
        m_telemetry.addData("servo error", m_frontLeftModule.getM_error());
        m_telemetry.addData("is motor flipped", m_frontLeftModule.getMotorFlip());
        m_telemetry.addData("heading", m_poseSupplier.get().getHeading());

        m_telemetry.addData("Raw Analog angle", m_frontLeftModule.getRawAngle());
        m_telemetry.addData("servo Power", m_frontLeftModule.getServoPower());
        m_telemetry.addData("P: ", m_frontLeftModule.getP());
        m_telemetry.update();

        if(m_PathFollowingEnabled)
        {
            update();
        }
    }
}