package org.firstinspires.ftc.teamcode.Test;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "BioBuzz Robo Centric", group = "Test")
public class BioBuzzRoboCentric extends LinearOpMode {

    // --- Mecanum Drive Configuration (Gamepad 1) ---
    public static String LEFT_FRONT_NAME = "leftFront";
    public static String LEFT_BACK_NAME = "leftBack";
    public static String RIGHT_FRONT_NAME = "rightFront";
    public static String RIGHT_BACK_NAME = "rightBack";
    public static double DRIVE_POWER_SCALE = 1.0;       // 50% max drive power (Editable in Dashboard)

    // --- Intake Configuration (Gamepad 1) ---
    public static String INTAKE_NAME = "intake";
    public static double INTAKE_POWER = 1.0;

    // --- Axon Gate & Shifter Servo Configuration (Gamepad 2) ---
    public static double OPEN_POSITION = 0.4;           // Gate Open position
    public static double CLOSED_POSITION = 0.7;        // Gate Closed position
    public static String GATE_SERVO = "gateServo";

    public static double SHIFTER_OPEN_POSITION = 0.25;     // Shifter Open position
    public static double SHIFTER_CLOSED_POSITION = 0.95;   // Shifter Closed position
    public static String SHIFTER_SERVO = "shifterServo";

    // --- Turret Motor Configuration (Gamepad 2) ---
    public static String TURRET_NAME = "turret";
    public static double TURRET_TICKS_PER_REV = 537.7;     // GoBILDA 312 RPM output encoder resolution (537.7 PPR)
    public static double TURRET_GEAR_RATIO = 8.0;          // 1:8 motor-to-turret gear ratio
    public static double TURRET_MAX_POWER = 0.8;           // Power limit for turret movement
    public static double TURRET_MAX_ANGLE_DEG = 90.0;      // +/- 90 degrees limit from center home
    public static double TURRET_INCREMENT_SPEED = 2.0;     // Degrees per loop cycle for stick speed control
    public static boolean TURRET_REVERSED = false;         // Set true if turret rotates opposite

    // --- Vision & AprilTag Configuration ---
    public static String WEBCAM_NAME = "Webcam 1";
    public static double TURRET_AUTO_AIM_GAIN = 0.2;       // Tracking Gain for Turret Auto-Aim

    // AprilTag IDs (4 per alliance side: 2 Left, 2 Right)
    public static int BLUE_LEFT_TAG_1 = 11;
    public static int BLUE_LEFT_TAG_2 = 12;
    public static int BLUE_RIGHT_TAG_1 = 13;
    public static int BLUE_RIGHT_TAG_2 = 14;

    public static int RED_LEFT_TAG_1 = 15;
    public static int RED_LEFT_TAG_2 = 16;
    public static int RED_RIGHT_TAG_1 = 17;
    public static int RED_RIGHT_TAG_2 = 18;


    // --- GoBILDA 6000 RPM Flywheel Motors Configuration (Gamepad 2) ---
    public static double FLYWHEEL_OPEN_RPM = 2600.0;     // Target RPM when shifter is OPEN (0.25)
    public static double FLYWHEEL_CLOSED_RPM = 2350.0;   // Target RPM when shifter is CLOSED (0.95)
    public static double TICKS_PER_REV = 28.0;          // GoBILDA 6000 RPM motor encoder resolution (28 PPR)
    public static boolean MOTOR_2_REVERSED = true;      // Set true if motor 2 spins opposite to motor 1
    public static boolean FLYWHEEL_ENABLED = true;      // Toggle flywheel motors ON/OFF

    public static String MOTOR_1_NAME = "flyWheel1";
    public static String MOTOR_2_NAME = "flyWheel2";

    public enum IntakeMode {
        STOPPED,
        FORWARD,
        REVERSE
    }

    public enum TargetTagSide {
        NONE,
        LEFT,
        RIGHT
    }

    // State variables
    private boolean isGateOpen = false;                 // false = Closed (0.7), true = Open (0.4)
    private boolean isShifterOpen = false;              // false = Closed (0.7), true = Open (0.4)
    private double targetTurretAngle = 0.0;             // Target angle in degrees (-90.0 to +90.0)
    private IntakeMode intakeMode = IntakeMode.STOPPED;

    // Alliance & Auto-Aim State variables
    protected boolean isRedAlliance = false;             // False = Blue Alliance, True = Red Alliance
    private TargetTagSide targetTagSide = TargetTagSide.NONE;
    private boolean autoAimActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---

        // 1. Mecanum Drive Motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK_NAME);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK_NAME);

        // Reverse left side motors so positive power moves forward
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run drive motors without encoder for direct stick power control
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Intake Motor
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. Axon Gate & Shifter Servos
        Servo axonGateServo = hardwareMap.get(Servo.class, GATE_SERVO);
        Servo shifterServo = hardwareMap.get(Servo.class, SHIFTER_SERVO);

        // 4. Shooter Motors (Master / Follower to prevent PID fighting & heating)
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Master uses closed-loop velocity PID
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Follower mimics master power directly

        // Set shooter motors to FLOAT so flywheels coast down smoothly when stopped
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set initial gate and shifter state to closed
        isGateOpen = false;
        isShifterOpen = false;
        axonGateServo.setPosition(CLOSED_POSITION);
        shifterServo.setPosition(SHIFTER_CLOSED_POSITION);

        // 5. Turret Motor (GoBILDA 312 RPM, 1:8 Gear Ratio)
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_MAX_POWER);

        // 6. AprilTag Vision Processor Initialization
        AprilTagProcessor aprilTagProcessor = null;
        VisionPortal visionPortal = null;
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(true)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(new Size(640, 480))
                    .build();
        } catch (Exception e) {
            telemetry.addData("Vision Warning", "Webcam 1 not found or failed to initialize.");
        }

        // Wait for Start with Alliance Color toggle (X = Blue, B = Red)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x || gamepad2.x) {
                isRedAlliance = false;
            } else if (gamepad1.b || gamepad2.b) {
                isRedAlliance = true;
            }

            telemetry.addData("Status", "Initialized BioBuzz Robo Centric successfully");
            telemetry.addData("Alliance Color", isRedAlliance ? "RED ALLIANCE (Press X for Blue)" : "BLUE ALLIANCE (Press B for Red)");
            telemetry.addData("Gate State", "CLOSED (%.2f)", CLOSED_POSITION);
            telemetry.addData("Shifter State", "CLOSED (%.2f)", SHIFTER_CLOSED_POSITION);
            telemetry.addData("Turret State", "CENTER HOME (0°)");
            telemetry.addData("Gamepad 1", "Mecanum Drive (Sticks) | Intake (A: Reverse, B: Forward)");
            telemetry.addData("Gamepad 2", "Gate (B/X) | Shifter (A) | Shooter (Y) | Auto-Aim (LB: Left, RB: Right)");
            telemetry.update();
            sleep(20);
        }

        // Edge detection & change trackers
        boolean lastIntakeAState = false;
        boolean lastIntakeBState = false;
        boolean lastShifterAState = false;
        boolean lastShooterYState = false;
        double lastIntakePower = Double.NaN; // Force initial setPower call

        while (opModeIsActive()) {

            // ==========================================
            // GAMEPAD 1: MECANUM DRIVE & INTAKE
            // ==========================================

            // --- Mecanum Drive Control ---
            double y = -gamepad1.left_stick_y;  // Forward / Backward
            double x = gamepad1.left_stick_x;   // Strafe Left / Right
            double rx = gamepad1.right_stick_x; // Turn Left / Right

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double leftFrontPower = ((y + x + rx) / denominator) * DRIVE_POWER_SCALE;
            double leftBackPower = ((y - x + rx) / denominator) * DRIVE_POWER_SCALE;
            double rightFrontPower = ((y - x - rx) / denominator) * DRIVE_POWER_SCALE;
            double rightBackPower = ((y + x - rx) / denominator) * DRIVE_POWER_SCALE;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // --- Intake Control (Gamepad 1 - Button A: Reverse, Button B: Forward) ---
            if (gamepad1.a && !lastIntakeAState) {
                intakeMode = (intakeMode == IntakeMode.REVERSE) ? IntakeMode.STOPPED : IntakeMode.REVERSE;
            }
            lastIntakeAState = gamepad1.a;

            if (gamepad1.b && !lastIntakeBState) {
                intakeMode = (intakeMode == IntakeMode.FORWARD) ? IntakeMode.STOPPED : IntakeMode.FORWARD;
            }
            lastIntakeBState = gamepad1.b;

            double intakePower = 0.0;
            if (intakeMode == IntakeMode.FORWARD) {
                intakePower = INTAKE_POWER;
            } else if (intakeMode == IntakeMode.REVERSE) {
                intakePower = -INTAKE_POWER;
            }

            // Only send power command to hardware when power state changes
            if (intakePower != lastIntakePower) {
                intake.setPower(intakePower);
                lastIntakePower = intakePower;
            }


            // ==========================================
            // GAMEPAD 2: GATE SERVO, SHIFTER SERVO & SHOOTER
            // ==========================================

            // --- Axon Gate Servo Control (Gamepad 2: B = Open, X = Close) ---
            if (gamepad2.b) {
                isGateOpen = true;   // Button B = Open Gate
            } else if (gamepad2.x) {
                isGateOpen = false;  // Button X = Close Gate
            }

            double targetGatePos = isGateOpen ? OPEN_POSITION : CLOSED_POSITION;
            axonGateServo.setPosition(targetGatePos);

            // --- Shifter Servo Control (Gamepad 2: A = Toggle) ---
            if (gamepad2.a && !lastShifterAState) {
                isShifterOpen = !isShifterOpen;
            }
            lastShifterAState = gamepad2.a;

            double targetShifterPos = isShifterOpen ? SHIFTER_OPEN_POSITION : SHIFTER_CLOSED_POSITION;
            shifterServo.setPosition(targetShifterPos);

            // --- Auto-Aim Target Selection (Gamepad 2: LB = Left Tag, RB = Right Tag, D-Pad Down = Cancel) ---
            if (gamepad2.left_bumper) {
                targetTagSide = TargetTagSide.LEFT;
                autoAimActive = true;
            } else if (gamepad2.right_bumper) {
                targetTagSide = TargetTagSide.RIGHT;
                autoAimActive = true;
            } else if (gamepad2.dpad_down) {
                autoAimActive = false;
                targetTagSide = TargetTagSide.NONE;
            }

            // --- Turret Control (Auto-Aim or Manual Joystick X) ---
            AprilTagDetection targetDetection = null;
            if (autoAimActive && targetTagSide != TargetTagSide.NONE && aprilTagProcessor != null) {
                targetDetection = findTargetAprilTag(aprilTagProcessor, isRedAlliance, targetTagSide);
                if (targetDetection != null && targetDetection.ftcPose != null) {
                    double bearingError = targetDetection.ftcPose.bearing;
                    targetTurretAngle += bearingError * TURRET_AUTO_AIM_GAIN;
                    targetTurretAngle = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, targetTurretAngle));
                }
            } else if (Math.abs(gamepad2.left_stick_x) > 0.05) {
                autoAimActive = false;
                targetTurretAngle += gamepad2.left_stick_x * TURRET_INCREMENT_SPEED;
                targetTurretAngle = Math.max(-TURRET_MAX_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, targetTurretAngle));
            } else if (gamepad2.dpad_left) {
                autoAimActive = false;
                targetTurretAngle = -TURRET_MAX_ANGLE_DEG; // -90° (Preset Left)
            } else if (gamepad2.dpad_right) {
                autoAimActive = false;
                targetTurretAngle = TURRET_MAX_ANGLE_DEG;  // +90° (Preset Right)
            } else if (gamepad2.dpad_up) {
                autoAimActive = false;
                targetTurretAngle = 0.0;                    // 0° (Preset Center Home)
            }

            double ticksPerDegree = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;
            int targetTurretTicks = (int) Math.round(targetTurretAngle * ticksPerDegree);
            turret.setTargetPosition(targetTurretTicks);
            turret.setPower(TURRET_MAX_POWER);

            double currentTurretAngle = turret.getCurrentPosition() / ticksPerDegree;

            // --- Shooter Flywheel Control (Gamepad 2) ---
            if (gamepad2.y && !lastShooterYState) {
                FLYWHEEL_ENABLED = !FLYWHEEL_ENABLED;
            }
            lastShooterYState = gamepad2.y;

            shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            double currentTargetRPM = isShifterOpen ? FLYWHEEL_OPEN_RPM : FLYWHEEL_CLOSED_RPM;
            double targetTPS = FLYWHEEL_ENABLED ? (currentTargetRPM * TICKS_PER_REV) / 60.0 : 0.0;
            shooter1.setVelocity(targetTPS);
            
            // Follower Motor 2 tracks Motor 1 power directly to prevent PID competition and motor overheating
            shooter2.setPower(shooter1.getPower());

            double actualTPS1 = shooter1.getVelocity();
            double actualTPS2 = shooter2.getVelocity();
            double actualRPM1 = (actualTPS1 * 60.0) / TICKS_PER_REV;
            double actualRPM2 = (actualTPS2 * 60.0) / TICKS_PER_REV;


            // ==========================================
            // TELEMETRY
            // ==========================================
            telemetry.addData("--- Alliance ---", isRedAlliance ? "RED ALLIANCE" : "BLUE ALLIANCE");
            telemetry.addData("--- Gamepad 1 Status ---", "");
            telemetry.addData("Mecanum Drive Powers", "LF:%.2f LB:%.2f RF:%.2f RB:%.2f",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Intake (G1.A: Reverse | G1.B: Forward)", "%s (%.2f)", intakeMode.name(), intakePower);

            telemetry.addData("--- Gamepad 2 Status ---", "");
            telemetry.addData("Gate Servo (G2.B: Open | G2.X: Close)", isGateOpen ? "OPEN (%.3f)" : "CLOSED (%.3f)", targetGatePos);
            telemetry.addData("Shifter Servo (G2.A: Toggle)", isShifterOpen ? "OPEN (%.3f)" : "CLOSED (%.3f)", targetShifterPos);
            
            if (autoAimActive) {
                if (targetDetection != null && targetDetection.ftcPose != null) {
                    telemetry.addData("Turret Auto-Aim", "[LOCKED] Tag #%d | Bearing: %.1f° | Range: %.1fin",
                            targetDetection.id, targetDetection.ftcPose.bearing, targetDetection.ftcPose.range);
                } else {
                    telemetry.addData("Turret Auto-Aim", "[SEARCHING] Side: %s (No Tag Visible)", targetTagSide.name());
                }
            } else {
                telemetry.addData("Turret Auto-Aim", "OFF (Manual Stick Control)");
            }

            telemetry.addData("Turret Angle (G2.LeftStickX / Auto-Aim)", "Target: %.1f° | Current: %.1f° (%d ticks)",
                    targetTurretAngle, currentTurretAngle, turret.getCurrentPosition());
            telemetry.addData("Shooter Flywheel (G2.Y)", FLYWHEEL_ENABLED ? "RUNNING (Target: %.0f RPM)" : "STOPPED", currentTargetRPM);
            telemetry.addData("Shooter Motors Measured", "M1: %.1f RPM | M2: %.1f RPM", actualRPM1, actualRPM2);

            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Helper method to find the targeted AprilTag detection from AprilTagProcessor.
     */
    private AprilTagDetection findTargetAprilTag(AprilTagProcessor processor, boolean isRed, TargetTagSide side) {
        if (processor == null) return null;
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        int primaryId;
        int secondaryId;

        if (!isRed) { // Blue Alliance
            primaryId = (side == TargetTagSide.LEFT) ? BLUE_LEFT_TAG_1 : BLUE_RIGHT_TAG_1;
            secondaryId = (side == TargetTagSide.LEFT) ? BLUE_LEFT_TAG_2 : BLUE_RIGHT_TAG_2;
        } else { // Red Alliance
            primaryId = (side == TargetTagSide.LEFT) ? RED_LEFT_TAG_1 : RED_RIGHT_TAG_1;
            secondaryId = (side == TargetTagSide.LEFT) ? RED_LEFT_TAG_2 : RED_RIGHT_TAG_2;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (detection.id == primaryId || detection.id == secondaryId) {
                    return detection;
                }
            }
        }
        return null;
    }
}
