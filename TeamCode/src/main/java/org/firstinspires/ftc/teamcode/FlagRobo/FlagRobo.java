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

package org.firstinspires.ftc.teamcode.FlagRobo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "FlagBot", group = "Memorial Day Event")

public class FlagRobo extends LinearOpMode {

    // Define class members

//    public findCandy candyPresent = null;

    toPose moveto =new toPose(this);

    @Override
    public void runOpMode() {

        // Wait for the start button
        telemetry.addData(">", "Press Start to Activate the robot.");
        telemetry.update();
//        candyPresent = new findCandy(this);
        boolean runAuto = false;
        moveto.initialize();
//        candyPresent.Initialize();

        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.a) {
                runAuto = true;
            }

            moveto.home();
            moveto.stand();
            moveto.home();
            moveto.show();

            if (gamepad1.left_bumper) {
                //toPose(standPose);
                moveto.delivery();
            }
            if (gamepad1.y) {
                moveto.pick();
            }
            if (gamepad1.x) {
                moveto.setCloseClaw();
            }
            if (gamepad1.b) {
                moveto.setOpenClaw();
            }

            if (gamepad1.dpad_left)
            {
                runAuto = false;
                moveto.toPose(moveto.pickPose);
                moveto.pickPose.basePose=moveto.pickPose.basePose+.01;
                sleep(10);
            }
            if (gamepad1.dpad_right)
            {
                runAuto = false;
                moveto.toPose(moveto.pickPose);
                moveto.pickPose.basePose=moveto.pickPose.basePose-.01;
                sleep(10);
            }
            if (gamepad1.dpad_down)
            {
                runAuto = false;
                moveto.toPose(moveto.pickPose);
                moveto.pickPose.tipPose=moveto.pickPose.tipPose+.01;
                sleep(10);
            }

            if (gamepad1.dpad_down)
            {
                runAuto = false;
                moveto.toPose(moveto.pickPose);
                moveto.pickPose.tipPose=moveto.pickPose.tipPose-.01;
                sleep(10);
            }
        }
        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

        //toPose(homePose);
    }






}


