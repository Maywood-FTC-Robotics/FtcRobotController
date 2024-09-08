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
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.io.IOException;

/*
Autonomous OpMode script using Command-based Robot
 */
@Photon //use PhotonFTC
@Autonomous(name="Blue: Right_Auton", group="Blue")
//@Disabled
public class BlueRight_Auton_OpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Clear out old mappings, command references and subsystems from any previously run opModes
        CommandScheduler.getInstance().reset();

        //Instantiate the robot
        RobotContainer m_robot = null;
        try {
            m_robot = new RobotContainer(
                    hardwareMap,
                    telemetry,
                    gamepad1,
                    gamepad2,
                    Constants.OpModeType.AUTO,
                    "AutonTest2");
        } catch (IOException e) {
            e.printStackTrace();
        }

        //Wait for driver to press PLAY
        waitForStart();

        // Run the robot until the end of the match (or until the driver presses STOP)
        while (opModeIsActive() && !isStopRequested())
        {
            m_robot.run();
        }

        //Save out the Pose and alliance-specific heading offset for Field-centric Teleop
        PoseStorage.currentPose = m_robot.getRobotPose();
        PoseStorage.allianceHeadingOffset = Constants.BLUE_HEADING_OFFSET;
    }
}
