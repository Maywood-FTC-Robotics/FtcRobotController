package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    VisionPortal m_visionPortal;
    AprilTagProcessor m_aprilTagProcessor;
    AprilTagPoseFtc m_ftcPose;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;

        m_aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005) // These lens calibrations are specific to Arducam OV9281!!!
                .build();

        m_visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(m_aprilTagProcessor)
                .setCameraResolution(new Size(640, 480)) //change to 320X240 for higher fps
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //.enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();


        m_visionPortal.setProcessorEnabled(m_aprilTagProcessor, true);
        m_ftcPose = null;
        //output the OpenCV processed image from the webcam to the FTCDashboard
        //FtcDashboard.getInstance().startCameraStream(m_visionPortal, 30);
    }

    @Override
    public void periodic()
    {
        m_ftcPose = null;
        //m_telemetry.addData("FPS", m_visionPortal.getFps());
        for (AprilTagDetection detection : m_aprilTagProcessor.getDetections()) {
            m_ftcPose = detection.ftcPose;
//            m_telemetry.addData("APRILTAG", detection.id);
//            m_telemetry.addData("ftcX", detection.ftcPose.x);
//            m_telemetry.addData("ftcY", detection.ftcPose.y);
//            m_telemetry.addData("ftcR", detection.ftcPose.range);
//            m_telemetry.addData("ftcBearing", detection.ftcPose.bearing);
        }
//        m_telemetry.update();
////        if(m_showStage) {
////            m_telemetry.addData("Location:", m_colorDetectPipeline.getLocation());
////            m_telemetry.update();
////        }
    }

//    public void disablePipeline()
//    {
//        m_webcam.stopStreaming();
//        m_showStage = Boolean.FALSE;
//    }

    public AprilTagPoseFtc getFtcPose()
    {
        return m_ftcPose;
    }

    public int getLocation(){
        return 0;
    }
}
