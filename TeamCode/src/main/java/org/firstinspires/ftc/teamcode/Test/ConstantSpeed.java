package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
@TeleOp(name = "Constant Velocity Motor Drive", group = "Examples")
public class ConstantSpeed extends LinearOpMode {

    // Motor configuration
    private DcMotorEx motor_inTake,motor_conveyor,motor_Shooter,motor_Indexor;

    // Target velocity in ticks per second
    public static double TARGET_VELOCITY_TICKS_PER_SECOND = 1600;
    public static int TARGET_POSITION_INEXOR = -4800; // Change this value as needed;

    // PIDF coefficients (tune these for your motor)
    public static PIDFCoefficients VELOCITY_PIDF = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);
    @Override
    public void runOpMode() {
        // Initialize motor
        motor_Shooter = hardwareMap.get(DcMotorEx.class, "motor_Shooter");
        motor_Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_Indexor = hardwareMap.get(DcMotorEx.class, "motor_Indexor");
        motor_Indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motor_inTake = hardwareMap.get(DcMotorEx.class, "motor_inTake");
        motor_conveyor = hardwareMap.get(DcMotorEx.class, "motor_conveyor");
        //motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, VELOCITY_PIDF);
        //motor.setDirection();

        telemetry.addLine("Ready to run at constant velocity");
        telemetry.update();

        waitForStart();
        motor_Shooter.setVelocity(TARGET_VELOCITY_TICKS_PER_SECOND);

        while (opModeIsActive()) {
            // Set motor velocity

            if(gamepad1.a)
            {
                motor_conveyor.setPower(1);
                motor_inTake.setPower(1);
            }

            if(gamepad1.y)
            {
                double motorSpeed = motor_Shooter.getVelocity();

                // Display current velocity
                telemetry.addData("Target Velocity", TARGET_VELOCITY_TICKS_PER_SECOND);
                telemetry.addData("Actual Velocity",motorSpeed );
                telemetry.update();
            }
            else {
               // motor_inTake.setVelocity(0);
            }

            if(gamepad1.x)
            {
                motor_Indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_Indexor.setTargetPosition(TARGET_POSITION_INEXOR);
                motor_Indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_Indexor.setPower(1); // Adjust power as needed
                // Wait for the motor to reach the target
                while (opModeIsActive() && motor_Indexor.isBusy()) {
                    telemetry.addData("Current Position", motor_Indexor.getCurrentPosition());
                    telemetry.update();
                }
                // Stop the motor
                motor_Indexor.setPower(0);
            }

        }

        // Stop motor when OpMode ends
    }
}
