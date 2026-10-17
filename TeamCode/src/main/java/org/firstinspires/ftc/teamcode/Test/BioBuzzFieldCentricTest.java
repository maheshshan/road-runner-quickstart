package org.firstinspires.ftc.teamcode.Test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
public class BioBuzzFieldCentricTest extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime loopBraker = new ElapsedTime();

    private double headingError  = 0,
                    dPadAngle =0.0,
                    lastError=0.0,
                    integralSum=0.0,
                    TagHeading=0;
    boolean dPadPressed=false;
    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  turnSpeed     = 0;


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static double    HEADING_THRESHOLD       = 0.0175,    // How close must the heading get to the target before moving to next step.
                            P_TURN_GAIN             = 0.7, // Larger is more responsive, but also less stable.
                            I_TURN_GAIN             =0.01,
                            D_TURN_GAIN             =0.1;
    Pose2d pose;
    MecanumDrive drive;

    Vision vision;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(DriveConstants.transPositionX, DriveConstants.transPositionY,DriveConstants.transHeading));

        vision = new Vision(this);
        vision.initialize();

        waitForStart();

            while (opModeIsActive()) {
                dPadAngle =0.0;
                dPadPressed=false;

                if(gamepad1.dpad_up) {
                    dPadAngle = 0;
                    dPadPressed=true;
                } else if (gamepad1.dpad_right) {
                    dPadAngle = Math.toRadians(-90);
                    dPadPressed=true;
                }else if(gamepad1.dpad_down) {
                    dPadAngle = Math.toRadians(180);
                    dPadPressed=true;
                }else if(gamepad1.dpad_left) {
                    dPadAngle = Math.toRadians(90);
                    dPadPressed=true;
                }

                if(gamepad1.a){
                    drive.updatePoseEstimate();
                    pose = drive.localizer.getPose();
                    Actions.runBlocking(
                            new SequentialAction(
                    drive.actionBuilder(pose)
                            .turn(Math.toRadians(90))
                            .build()
                            )
                    );
                }

                if( dPadPressed){
                    turnToAngle(dPadAngle);
                }

                if(gamepad1.x){
                    TagHeading=vision.find();
                    drive.localizer.setPose(new Pose2d(0,0,-Math.toRadians(TagHeading)));
                }

                double botHeading = getHeading();

                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;

                // Rotate the movement direction counter to the bot's rotation
                double rotY = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotX = y * Math.cos(botHeading) + x *  Math.sin(botHeading);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                rotX,
                                rotY
                        ),
                        rx
                ));

                telemetry.addData("TagHeading",TagHeading);
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

    }


    public void turnToAngle(double Angle)
    {
        headingError = Angle - getHeading();
        timer.reset();
        loopBraker.reset();
        while (opModeIsActive()&&Math.abs(headingError) > HEADING_THRESHOLD&&loopBraker.milliseconds()<1000) {
            headingError = Angle - getHeading();

            turnSpeed = PIDControl(Angle);

            /*headingError = Angle - getHeading();
            while (headingError > Math.PI)  headingError -= (Math.PI*2);
            while (headingError <= -Math.PI) headingError += (Math.PI*2);

            turnSpeed = Range.clip(headingError*P_TURN_GAIN, -1, 1);
*/
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0,
                            0
                    ),
                    turnSpeed
            ));
        }
    }

    public double getHeading()
    {
        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();
        return pose.heading.toDouble();
    }

    public double PIDControl(double Angle)
    {
        headingError = Angle-getHeading();
        while (headingError > Math.PI)  headingError -= (Math.PI*2);
        while (headingError <= -Math.PI) headingError += (Math.PI*2);

        integralSum +=headingError+timer.seconds();
        if (Math.abs(headingError) < HEADING_THRESHOLD) {
            integralSum = 0;
        }

/*        if (timer.milliseconds() > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }*/
        double drivative = (headingError-lastError)/timer.seconds();
        lastError=headingError;

        timer.reset();
        double output=(headingError*P_TURN_GAIN)+(drivative*D_TURN_GAIN)+(integralSum*I_TURN_GAIN);
        output = Range.clip(output, -1, 1);
        return output;
    }
}
