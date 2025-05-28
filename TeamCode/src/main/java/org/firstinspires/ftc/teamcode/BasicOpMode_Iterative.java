package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {

    public static int targetSlidePitchPosition = 0;
    public static int targetSlideRetractionPosition = 0;

    public static float adjustmentsClawRoll = 0.5f;
    public static float adjustmentsWristPitch = 0.5f;

    public static int adjustmentsSlidePitch = 20;

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
        slidePitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRetraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRetraction.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        boolean prevToggleButtonState = false;
        targetSlidePitchPosition = slidePitch.getCurrentPosition();
        targetSlideRetractionPosition = slidePitch.getCurrentPosition();
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

            // Calibration: R3 + L3 to set vertical position calibration
            boolean currentToggleButtonState = gamepad1.right_stick_button && gamepad1.left_stick_button;
            if (currentToggleButtonState && !prevToggleButtonState) {
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

            // Slide pitch control (RUN_TO_POSITION)
            if (gamepad1.y) {
                targetSlidePitchPosition += adjustmentsSlidePitch;
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= adjustmentsSlidePitch;
            }
            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Slide retraction control (RUN_TO_POSITION, no software limit)
            if (gamepad1.right_trigger > 0.1) {
                targetSlideRetractionPosition += 10;
            } else if (gamepad1.left_trigger > 0.1) {
                targetSlideRetractionPosition -= 10;
            }
            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Wrist pitch control (CRServo)
            if (gamepad1.dpad_up) {
                wristPitch.setPower(adjustmentsWristPitch);
            } else if (gamepad1.dpad_down) {
                wristPitch.setPower(-adjustmentsWristPitch);
            } else {
                wristPitch.setPower(0);
            }

            // Claw roll (CRServo)
            if (gamepad1.dpad_left) {
                clawRoll.setPower(adjustmentsClawRoll);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPower(-adjustmentsClawRoll);
            } else {
                clawRoll.setPower(0);
            }

            // Claw grip (CRServo)
            if (gamepad1.right_bumper) {
                clawGrip.setPower(0.8);
                gamepad1.rumble(1);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPower(-0.8);
            } else {
                clawGrip.setPower(0);
                gamepad1.stopRumble();
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Vertical Calibration", verticalPositionCalibration == null ? "Not calibrated" : verticalPositionCalibration);
            telemetry.addData("SlidePitch Pos", slidePitch.getCurrentPosition());
            telemetry.addData("SlideRetraction Target", targetSlideRetractionPosition);
            telemetry.addData("SlideRetraction Pos", slideRetraction.getCurrentPosition());
            telemetry.addData("Slide Retraction Limited", "No limit applied");
            telemetry.update();
        }
    }
}
