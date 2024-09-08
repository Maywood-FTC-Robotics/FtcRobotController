/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.io.IOException;

/*
TeleOp OpMode script using Command-based Robot
 */

@Photon
@TeleOp(name="TeleOp_OpMode", group="TeleOp")
//@Disabled
public class TeleOp_OpMode extends LinearOpMode {
    ElapsedTime m_elapsedTime = new ElapsedTime();
    MovingStatistics m_movingStats = new MovingStatistics(50);
    @Override
    public void runOpMode() {

        //Clear out old mappings, command references and subsystems from any previously run opModes
        CommandScheduler.getInstance().reset();

        //Instantiate the robot (creates all subsystems and button mappings)
        RobotContainer m_robot = null;
        try {
            m_robot = new RobotContainer(
                    hardwareMap,
                    telemetry,
                    gamepad1,
                    gamepad2,
                    Constants.OpModeType.TELEOP);
        } catch (IOException e) {
            e.printStackTrace();
        }

        m_robot.restoreFromPoseStorage(PoseStorage.currentPose,
                PoseStorage.allianceHeadingOffset);

        //Wait for driver to press PLAY and then STOP
        waitForStart();

        // Run the robot until the end of the match (or until the driver presses STOP)
        while (opModeIsActive() && !isStopRequested())
        {
            m_elapsedTime.reset();
            m_robot.run();
            m_movingStats.add(m_elapsedTime.milliseconds());
            RobotLog.d(String.format("LOOPTIME: %.1f; MEAN: %.1f; STDDEV: %.1f", m_elapsedTime.milliseconds(), m_movingStats.getMean(), m_movingStats.getStandardDeviation()));
//            RobotLog.d(String.format("LOOPTIME: %.1f", m_elapsedTime.milliseconds()));
//            telemetry.addData("LoopTime", m_movingStats.getMean());
//            telemetry.update();
        }

        //When we end Teleop, reset the PoseStorage to zero (in the case of restarting Teleop)
        PoseStorage.currentPose = new Pose2d();
        PoseStorage.allianceHeadingOffset = 0.0; //No Heading Offset after TeleOp
    }
}
