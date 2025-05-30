package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {

    // Incremental slide position targets
    public static int targetSlidePitchPosition = 0;
    public static int targetSlideRetractionPosition = 0;

    // Preset positions for slide, wrist, and claw grip
    public static int slidePitchIdle = 0;
    public static int slideRetractionIdle = 0;
    public static double wristPitchIdle = 0.5;
    public static double clawGripIdle = 0.5;

    public static int slidePitchCollect = 100;
    public static int slideRetractionCollect = -500;
    public static double wristPitchCollect = 0.3;
    public static double clawGripCollect = 0.7;

    public static int slidePitchScore = 300;
    public static int slideRetractionScore = -1000;
    public static double wristPitchScore = 0.8;
    public static double clawGripScore = 0.2;

    public static float adjustmentsClawRoll = 0.5f;
    public static int adjustmentsSlidePitch = 20;
    public static double adjustmentsWristPitch = 0.02;
    public static double adjustmentsClawGrip = 0.02;

    // Limits for slideRetraction
    private static final int SLIDE_RETRACTION_MIN = -1920;
    private static final int SLIDE_RETRACTION_MAX = 40;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");
        Servo clawGrip = hardwareMap.servo.get("clawGrip");
        CRServo clawRoll = hardwareMap.crservo.get("clawRoll");

        // Directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePitch.setDirection(DcMotor.Direction.REVERSE);
        slidePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRetraction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRetraction.setDirection(DcMotor.Direction.FORWARD);
        slideRetraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRetraction.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        // Initialize targets
        targetSlidePitchPosition = slidePitch.getCurrentPosition();
        targetSlideRetractionPosition = slideRetraction.getCurrentPosition();

        // Initialize wrist and clawGrip positions (servo ranges 0 to 1)
        double wristPitchPos = 0.5;
        double clawGripPos = 0.5;

        while (opModeIsActive()) {
            // ---- Gamepad1: Drive + Incremental adjustments ----
            // Drive controls
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // Incremental slidePitch adjustments
            if (gamepad1.y) {
                targetSlidePitchPosition += adjustmentsSlidePitch;
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= adjustmentsSlidePitch;
            }

            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Incremental slideRetraction adjustments
            if (gamepad1.right_trigger > 0.1) {
                targetSlideRetractionPosition += 10;
            } else if (gamepad1.left_trigger > 0.1) {
                targetSlideRetractionPosition -= 10;
            }

            // Clamp slideRetraction target
            targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, SLIDE_RETRACTION_MAX);
            targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, SLIDE_RETRACTION_MIN);

            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Incremental wristPitch adjustments on gamepad1 dpad_up/down (optional)
            if (gamepad1.dpad_up) {
                wristPitchPos = Math.min(1.0, wristPitchPos + adjustmentsWristPitch);
            } else if (gamepad1.dpad_down) {
                wristPitchPos = Math.max(0.0, wristPitchPos - adjustmentsWristPitch);
            }

            // Incremental clawGrip adjustments on gamepad1 bumpers (optional)
            if (gamepad1.right_bumper) {
                clawGripPos = Math.min(1.0, clawGripPos + adjustmentsClawGrip);
            } else if (gamepad1.left_bumper) {
                clawGripPos = Math.max(0.0, clawGripPos - adjustmentsClawGrip);
            }

            wristPitch.setPosition(wristPitchPos);
            clawGrip.setPosition(clawGripPos);

            // ---- Gamepad2: Preset positions + ClawRoll control ----

            // Set IDLE preset (dpad_down)
            if (gamepad2.dpad_down) {
                targetSlidePitchPosition = slidePitchIdle;
                targetSlideRetractionPosition = slideRetractionIdle;
                wristPitchPos = wristPitchIdle;
                clawGripPos = clawGripIdle;
            }
            // Set COLLECT preset (dpad_left)
            if (gamepad2.left_bumper) {
                targetSlidePitchPosition = slidePitchCollect;
                targetSlideRetractionPosition = slideRetractionCollect;
                wristPitchPos = wristPitchCollect;
                clawGripPos = clawGripCollect;
            }
            // Set SCORE preset (dpad_right)
            if (gamepad2.right_bumper) {
                targetSlidePitchPosition = slidePitchScore;
                targetSlideRetractionPosition = slideRetractionScore;
                wristPitchPos = wristPitchScore;
                clawGripPos = clawGripScore;
            }

            // Update slide motors target and mode again for presets
            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Clamp slideRetraction target again after preset changes
            targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, SLIDE_RETRACTION_MAX);
            targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, SLIDE_RETRACTION_MIN);

            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Apply wristPitch and clawGrip positions from presets
            wristPitch.setPosition(wristPitchPos);
            clawGrip.setPosition(clawGripPos);

            // ClawRoll control (CRServo) gamepad2 dpad left/right
            if (gamepad1.dpad_left) {
                clawRoll.setPower(adjustmentsClawRoll);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPower(-adjustmentsClawRoll);
            } else {
                clawRoll.setPower(0);
            }

            // Telemetry for dashboard
            telemetry.addData("Status", "Running");
            telemetry.addData("SlidePitch Target", targetSlidePitchPosition);
            telemetry.addData("SlidePitch Current", slidePitch.getCurrentPosition());
            telemetry.addData("SlideRetraction Target", targetSlideRetractionPosition);
            telemetry.addData("SlideRetraction Current", slideRetraction.getCurrentPosition());
            telemetry.addData("WristPitch Position", wristPitchPos);
            telemetry.addData("ClawGrip Position", clawGripPos);
            telemetry.addData("ClawRoll Power", clawRoll.getPower());
            telemetry.update();
        }
    }
}
