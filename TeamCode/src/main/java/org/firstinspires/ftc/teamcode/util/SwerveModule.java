    package org.firstinspires.ftc.teamcode.util;

    import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Rotation2d;
    import com.arcrobotics.ftclib.command.SubsystemBase;
    import com.arcrobotics.ftclib.controller.PIDFController;
    import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
    import com.qualcomm.robotcore.hardware.AnalogInput;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.PIDCoefficients;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.Constants;


@Config
public class SwerveModule extends SubsystemBase{
    DcMotorEx m_driveMotor;
    CRServo m_turnServo;
    AnalogEncoder m_turnEncoder;

    private double motorPower;
    private double m_targetVelocity;
    private double m_turnOffset;
    private double m_currentAngle;
    private double m_targetAngle;
    private double m_previousAngle = 0;
    private double m_error;

    public double servoPower = 0;

    private Boolean isFlipped = false;
    public static double kp = 0.005;
    public static double ki = 0.0;
    public static double kd = 0.0;

    public static double p = 0.005;

    public static double antiCoaxialConstant = 0.1;

    public static double scale = 1000;
    PIDController controller = new PIDController(kp,ki,kd);

    public SwerveModule(DcMotorEx driveMotor, CRServo turnServo,
                        AnalogEncoder turnEncoder, double turnOffset){
        m_driveMotor = driveMotor;
        m_turnServo = turnServo;
        m_turnEncoder = turnEncoder;
        m_turnEncoder.setIS_INVERTED(true);
        m_turnOffset = turnOffset;
        controller.enableContinuousInput(0,359.999);
    }

    public void setState(SwerveModuleState state)
    {
        if(state.speedMetersPerSecond/Constants.MAX_VELOCITY  > 0.06){
            setPIDandMotorSpeed(state.angle.getDegrees() ,state.speedMetersPerSecond);
        } else {
            setPIDandMotorSpeed(m_previousAngle, 0);
        }
    }

    public void setPIDandMotorSpeed(double angle, double speed){
        m_previousAngle = m_targetAngle;
        m_targetAngle = angle;
        m_currentAngle = normalizeDegrees(m_turnEncoder.getAngle() + m_turnOffset);

        m_error = normalizeDegrees(Math.abs(m_targetAngle - m_currentAngle) + getFlipCorrection());
        // Handle flipping logic
        if (m_error > 90 && m_error < 270) {
            isFlipped = !isFlipped;
        }

        m_targetAngle = normalizeDegrees(m_targetAngle + getFlipCorrection());
         servoPower = controller.calculate(m_currentAngle, m_targetAngle);
//        if(servoPower != 0) {
//            activate(servoPower);
//        }

        m_turnServo.setPower(servoPower);

        motorPower = speed / Constants.MAX_VELOCITY;
        m_driveMotor.setPower(isFlipped? motorPower : -motorPower);

        // Ensure PID constants are set (if they need to be updated dynamically)
        controller.setPID(kp, ki, kd);

        }

        public void activate(double power){
            if(isAgainstCoaxial(power)){kp = p - (antiCoaxialConstant * motorPower)/scale;}
            else if(!isAgainstCoaxial(power)){kp = p + (antiCoaxialConstant * motorPower)/scale;}
            m_turnServo.setPower(power);
        }

        public Boolean isAgainstCoaxial(double power) {
            if (!isFlipped) {
                if (power > 0) {
                    return true;
                } else if (power < 0) {
                    return false;
                }
            } else if (isFlipped) {
                if (power > 0) {
                    return false;
                } else if (power < 0) {
                    return true;
                }
            }
            return false;
    }

            public double normalizeDegrees(double degrees){
            while(degrees > 360){degrees -= 360;}
            while(degrees < 0){degrees += 360;}
            return degrees;
        }

        public double getFlipCorrection(){
            if(isFlipped){
                return 180;
            }else{return 0;}
        }




        public double getCurrentTarget(){return m_targetAngle;}

        public double getServoAngle(){return m_currentAngle;}

        public double getServoPower(){return servoPower;}

    public double getP(){return kp;}

      public double getM_error(){return m_error;}

        public boolean getMotorFlip(){return isFlipped;}

        public double getRawAngle(){return m_turnEncoder.getAngle();}


    }

