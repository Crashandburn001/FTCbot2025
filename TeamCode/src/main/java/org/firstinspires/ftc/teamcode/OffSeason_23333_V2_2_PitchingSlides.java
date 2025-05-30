package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@Config
@TeleOp(name = "BasicOpMode_Iterative")
public class OffSeason_23333_V2_2_PitchingSlides extends LinearOpMode {

    public static int targetSlidePitchPosition = 0;
    public static int targetSlideRetractionPosition = 0;

    public static double wristPitchPos = 0.5;

    public static double adjustmentsWristPitch = 0.01;
    public static float adjustmentsClawRoll = 0.5f;
    public static int adjustmentsSlidePitch = 20;

    // Variables for Collect Set Pos
    public static int COLLECT_SLIDE_PITCH = 0;
    public static int COLLECT_SLIDE_RETRACTION = -1900;
    public static double COLLECT_WRIST_PITCH = 0.2;

    //Variables for SCORE Set Pos
    public static int SCORE_SLIDE_PITCH = 300;
    public static int SCORE_SLIDE_RETRACTION = -600;
    public static double SCORE_WRIST_PITCH = 0.8;

    //Variables for IDLE Set Pos
    public static int IDLE_SLIDE_PITCH = 0;
    public static int IDLE_SLIDE_RETRACTION = 0;
    public static double IDLE_WRIST_PITCH = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        CRServo clawRoll = hardwareMap.crservo.get("clawRoll");
        CRServo clawGrip = hardwareMap.crservo.get("clawGrip");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");

        slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePitch.setDirection(DcMotor.Direction.REVERSE);
        slideRetraction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRetraction.setDirection(DcMotor.Direction.FORWARD);

        slidePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRetraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRetraction.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        targetSlidePitchPosition = slidePitch.getCurrentPosition();
        targetSlideRetractionPosition = slideRetraction.getCurrentPosition();

        while (opModeIsActive()) {

            // Manual Slide Pitch control
            if (gamepad1.y) {
                targetSlidePitchPosition += adjustmentsSlidePitch;
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= adjustmentsSlidePitch;
            }

            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Manual Slide Retraction
            if ((gamepad1.right_trigger > 0.1)) {
                targetSlideRetractionPosition += 10;
            } else if ((gamepad1.left_trigger > 0.1)) {
                targetSlideRetractionPosition -= 10;
            }

            // Clamp retraction
            targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, 40);
            targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, -1920);

            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Manual wrist pitch (gamepad1)
            if (gamepad1.dpad_up) {
                wristPitchPos += adjustmentsWristPitch;
            } else if (gamepad1.dpad_down) {
                wristPitchPos -= adjustmentsWristPitch;
            }
            wristPitchPos = Math.max(0, Math.min(1, wristPitchPos));
            wristPitch.setPosition(wristPitchPos);

            // Claw roll (CRServo - gamepad1)
            if (gamepad1.dpad_left) {
                clawRoll.setPower(adjustmentsClawRoll);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPower(-adjustmentsClawRoll);
            } else {
                clawRoll.setPower(0);
            }

            // Claw grip (CRServo - gamepad1)
            if (gamepad1.right_bumper) {
                clawGrip.setPower(0.8);
                gamepad1.rumble(1);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPower(-0.8);
            } else {
                clawGrip.setPower(0);
                gamepad1.stopRumble();
            }

            // Set positions (gamepad2)
            if (gamepad2.a) { // COLLECT
                targetSlidePitchPosition = COLLECT_SLIDE_PITCH;
                targetSlideRetractionPosition = COLLECT_SLIDE_RETRACTION;
                wristPitchPos = COLLECT_WRIST_PITCH;
            } else if (gamepad2.y) { // SCORE
                targetSlidePitchPosition = SCORE_SLIDE_PITCH;
                targetSlideRetractionPosition = SCORE_SLIDE_RETRACTION;
                wristPitchPos = SCORE_WRIST_PITCH;
            } else if (gamepad2.b) { // IDLE
                targetSlidePitchPosition = IDLE_SLIDE_PITCH;
                targetSlideRetractionPosition = IDLE_SLIDE_RETRACTION;
                wristPitchPos = IDLE_WRIST_PITCH;
            }

            telemetry.addData("SlidePitch Pos", slidePitch.getCurrentPosition());
            telemetry.addData("SlideRetraction Pos", slideRetraction.getCurrentPosition());
            telemetry.addData("Target Pitch", targetSlidePitchPosition);
            telemetry.addData("Target Retract", targetSlideRetractionPosition);
            telemetry.addData("Wrist Pitch", wristPitchPos);
            telemetry.update();
        }
    }
}
