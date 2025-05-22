package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {

    private int targetSlidePitchPosition = 0;
    private int targetSlideRetractionPosition = 0;

    private final int SLIDE_HORIZONTAL_MAX_POSITION = 500; // tune this value
    private final int SLIDE_HORIZONTAL_MIN_POSITION = 0;

    // Calibration variables
    private Integer verticalPositionCalibration = null; // null = not calibrated yet
    private final int VERTICAL_POSITION_TOLERANCE = 20; // allowable encoder ticks tolerance

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        CRServo wristPitch = hardwareMap.crservo.get("wristPitch");
        CRServo clawRoll = hardwareMap.crservo.get("clawRoll");
        CRServo clawGrip = hardwareMap.crservo.get("clawGrip");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePitch.setDirection(DcMotor.Direction.REVERSE);

        slideRetraction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRetraction.setDirection(DcMotor.Direction.FORWARD);

        slidePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRetraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        if (isStopRequested()) return;

        // For toggle button press edge detection
        boolean prevToggleButtonState = false;

        while (opModeIsActive()) {

            // Drive control (unchanged)
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // --- Calibration: Detect R3 + L3 button combo press to set vertical position calibration ---
            boolean currentToggleButtonState = gamepad1.right_stick_button && gamepad1.left_stick_button;
            if (currentToggleButtonState && !prevToggleButtonState) {
                // Button combo just pressed - calibrate vertical position
                verticalPositionCalibration = slidePitch.getCurrentPosition();
                telemetry.addData("Calibration", "Vertical position calibrated at %d", verticalPositionCalibration);
            }
            prevToggleButtonState = currentToggleButtonState;

            // Determine if slidePitch is vertical or horizontal based on calibration
            boolean isSlideHorizontal = true; // default

            if (verticalPositionCalibration != null) {
                int currentPos = slidePitch.getCurrentPosition();
                int diff = Math.abs(currentPos - verticalPositionCalibration);
                if (diff <= VERTICAL_POSITION_TOLERANCE) {
                    isSlideHorizontal = false; // slide is vertical
                }
            }

            // --- Slide pitch control ---
            if (gamepad1.y) {
                targetSlidePitchPosition += 1; // move to position +1 for smooth PID control
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= 1;
            }
            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setPower(0.8);

            // --- Slide retraction control with software limits applied only if horizontal ---
            if (gamepad1.right_trigger > 0.1) {
                targetSlideRetractionPosition += 1;
            } else if (gamepad1.left_trigger > 0.1) {
                targetSlideRetractionPosition -= 1;
            }

            if (isSlideHorizontal) {
                if (targetSlideRetractionPosition > SLIDE_HORIZONTAL_MAX_POSITION) {
                    targetSlideRetractionPosition = SLIDE_HORIZONTAL_MAX_POSITION;
                } else if (targetSlideRetractionPosition < SLIDE_HORIZONTAL_MIN_POSITION) {
                    targetSlideRetractionPosition = SLIDE_HORIZONTAL_MIN_POSITION;
                }
            }
            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setPower(0.8);

            // --- Wrist pitch control (CRServo) ---
            if (gamepad1.dpad_up) {
                wristPitch.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                wristPitch.setPower(-0.5);
            } else {
                wristPitch.setPower(0);
            }

            // --- Claw roll (CRServo) ---
            if (gamepad1.dpad_left) {
                clawRoll.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPower(-0.5);
            } else {
                clawRoll.setPower(0);
            }

            // --- Claw grip (CRServo) ---
            if (gamepad1.right_bumper) {
                clawGrip.setPower(0.8);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPower(-0.8);
            } else {
                clawGrip.setPower(0);
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Vertical Calibration", verticalPositionCalibration == null ? "Not calibrated" : verticalPositionCalibration);
            telemetry.addData("SlidePitch Pos", slidePitch.getCurrentPosition());
            telemetry.addData("SlideRetraction Target", targetSlideRetractionPosition);
            telemetry.addData("SlideRetraction Pos", slideRetraction.getCurrentPosition());
            telemetry.addData("SlideRetraction Limited", isSlideHorizontal);
            telemetry.update();
        }
    }
}
