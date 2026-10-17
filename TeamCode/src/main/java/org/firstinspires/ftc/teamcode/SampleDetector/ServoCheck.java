/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.SampleDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;



//@Disabled
@TeleOp(name = "Concept: ServoCheck", group = "Tuning")
public class ServoCheck extends LinearOpMode
{
    toPoseSample moveto =new toPoseSample(this);


    @Override
    public void runOpMode()
    {
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        moveto.initialize();


        double ServoSpeed = 0.001;




        waitForStart();

        while (opModeIsActive())
        {

            if(gamepad1.right_stick_y!=0){
                moveto.crtPose.tipPose=moveto.crtPose.tipPose-(ServoSpeed*gamepad1.right_stick_y);
                moveto.setToPose();
            }

            if(gamepad1.right_stick_x!=0){
                moveto.crtPose.swivelPose=moveto.crtPose.swivelPose+(ServoSpeed*gamepad1.right_stick_x);
                moveto.setToPose();
            }

            if(gamepad1.left_stick_y!=0){
                moveto.crtPose.arm1Pose=moveto.crtPose.arm1Pose+(ServoSpeed*gamepad1.left_stick_y);
                moveto.crtPose.arm2Pose=moveto.crtPose.arm2Pose+(ServoSpeed*gamepad1.left_stick_y);
                moveto.setToPose();
            }
            if(gamepad1.dpad_down){
                moveto.crtPose.arm2Pose=moveto.crtPose.arm2Pose-ServoSpeed*.1;
                moveto.setToPose();
            }
            if(gamepad1.dpad_up){
                moveto.crtPose.arm2Pose=moveto.crtPose.arm2Pose+ServoSpeed*0.1;
                moveto.setToPose();
            }
            if(gamepad1.left_stick_x!=0){
                moveto.crtPose.basePose=moveto.crtPose.basePose+(ServoSpeed*gamepad1.left_stick_x);
                moveto.setToPose();
            }

            if(gamepad1.a) moveto.home();

            if(gamepad1.x) moveto.setCloseClaw();
            if(gamepad1.b) moveto.setOpenClaw();


            moveto.printCrtPose();

        }
    }
}
