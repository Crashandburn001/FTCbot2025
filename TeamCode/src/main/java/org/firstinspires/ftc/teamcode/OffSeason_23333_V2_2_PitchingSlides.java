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

    public static int COLLECT_SLIDE_PITCH = 0;
    public static int COLLECT_SLIDE_RETRACTION = -600;
    public static double COLLECT_WRIST_PITCH = 0.2;

    public static int SCORE_SLIDE_PITCH = 300;
    public static int SCORE_SLIDE_RETRACTION = -1200;
    public static double SCORE_WRIST_PITCH = 0.8;

    public static int IDLE_SLIDE_PITCH = 0;
    public static int IDLE_SLIDE_RETRACTION = 0;
    public static double IDLE_WRIST_PITCH = 0.5;

    public static int SPECIMEN_COLLECT_SLIDE_PITCH = 150;
    public static int SPECIMEN_COLLECT_SLIDE_RETRACTION = -700;
    public static double SPECIMEN_COLLECT_WRIST_PITCH = 0.3;

    public static int SPECIMEN_SCORE_SLIDE_PITCH = 450;
    public static int SPECIMEN_SCORE_SLIDE_RETRACTION = -1300;
    public static double SPECIMEN_SCORE_WRIST_PITCH = 0.85;

    // ArmState without enum
    public static final int STATE_IDLE = 0;
    public static final int STATE_MOVING_TO_COLLECT = 1;
    public static final int STATE_MOVING_TO_SCORE = 2;
    public static final int STATE_MOVING_TO_IDLE = 3;
    public static final int STATE_MOVING_TO_SPECIMEN_COLLECT = 4;
    public static final int STATE_MOVING_TO_SPECIMEN_SCORE = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        CRServo clawRoll = hardwareMap.crservo.get("clawRoll");
        CRServo clawGrip = hardwareMap.crservo.get("clawGrip");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        int armState = STATE_IDLE;
        boolean commandIssued = false;
        final int SLIDE_PITCH_TOLERANCE = 10;

        while (opModeIsActive()) {
            // Driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
            double fl = (drive + strafe + turn) / denominator;
            double fr = (drive - strafe - turn) / denominator;
            double bl = (drive - strafe + turn) / denominator;
            double br = (drive + strafe - turn) / denominator;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Manual slide pitch control
            if (gamepad1.y) targetSlidePitchPosition += adjustmentsSlidePitch;
            else if (gamepad1.x) targetSlidePitchPosition -= adjustmentsSlidePitch;

            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Manual slide retraction control
            if (gamepad1.right_trigger > 0.1) targetSlideRetractionPosition += 10;
            else if (gamepad1.left_trigger > 0.1) targetSlideRetractionPosition -= 10;

            targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, 40);
            targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, -1920);

            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Wrist pitch control
            if (gamepad1.dpad_up) wristPitchPos += adjustmentsWristPitch;
            else if (gamepad1.dpad_down) wristPitchPos -= adjustmentsWristPitch;

            wristPitchPos = Math.max(0, Math.min(1, wristPitchPos));
            wristPitch.setPosition(wristPitchPos);

            // Claw roll (CR servo)
            if (gamepad1.dpad_left) clawRoll.setPower(adjustmentsClawRoll);
            else if (gamepad1.dpad_right) clawRoll.setPower(-adjustmentsClawRoll);
            else clawRoll.setPower(0);

            // Claw grip (CR servo)
            if (gamepad1.right_bumper) {
                clawGrip.setPower(0.8);
                gamepad1.rumble(1);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPower(-0.8);
            } else {
                clawGrip.setPower(0);
                gamepad1.stopRumble();
            }

            // Arm states
            if (gamepad2.a && !commandIssued) {
                targetSlidePitchPosition = COLLECT_SLIDE_PITCH;
                armState = STATE_MOVING_TO_COLLECT;
                commandIssued = true;
            } else if (gamepad2.y && !commandIssued) {
                targetSlidePitchPosition = SCORE_SLIDE_PITCH;
                armState = STATE_MOVING_TO_SCORE;
                commandIssued = true;
            } else if (gamepad2.b && !commandIssued) {
                targetSlidePitchPosition = IDLE_SLIDE_PITCH;
                armState = STATE_MOVING_TO_IDLE;
                commandIssued = true;
            } else if (gamepad2.x && !commandIssued) {
                targetSlidePitchPosition = SPECIMEN_COLLECT_SLIDE_PITCH;
                armState = STATE_MOVING_TO_SPECIMEN_COLLECT;
                commandIssued = true;
            } else if (gamepad2.dpad_up && !commandIssued) {
                targetSlidePitchPosition = SPECIMEN_SCORE_SLIDE_PITCH;
                armState = STATE_MOVING_TO_SPECIMEN_SCORE;
                commandIssued = true;
            }

            if (!gamepad2.a && !gamepad2.y && !gamepad2.b && !gamepad2.x && !gamepad2.dpad_up)
                commandIssued = false;

            boolean slidePitchInPosition = Math.abs(slidePitch.getCurrentPosition() - targetSlidePitchPosition) < SLIDE_PITCH_TOLERANCE;

            switch (armState) {
                case STATE_MOVING_TO_COLLECT:
                    if (slidePitchInPosition) {
                        targetSlideRetractionPosition = COLLECT_SLIDE_RETRACTION;
                        wristPitchPos = COLLECT_WRIST_PITCH;
                        armState = STATE_IDLE;
                    }
                    break;
                case STATE_MOVING_TO_SCORE:
                    if (slidePitchInPosition) {
                        targetSlideRetractionPosition = SCORE_SLIDE_RETRACTION;
                        wristPitchPos = SCORE_WRIST_PITCH;
                        armState = STATE_IDLE;
                    }
                    break;
                case STATE_MOVING_TO_IDLE:
                    if (slidePitchInPosition) {
                        targetSlideRetractionPosition = IDLE_SLIDE_RETRACTION;
                        wristPitchPos = IDLE_WRIST_PITCH;
                        armState = STATE_IDLE;
                    }
                    break;
                case STATE_MOVING_TO_SPECIMEN_COLLECT:
                    if (slidePitchInPosition) {
                        targetSlideRetractionPosition = SPECIMEN_COLLECT_SLIDE_RETRACTION;
                        wristPitchPos = SPECIMEN_COLLECT_WRIST_PITCH;
                        armState = STATE_IDLE;
                    }
                    break;
                case STATE_MOVING_TO_SPECIMEN_SCORE:
                    if (slidePitchInPosition) {
                        targetSlideRetractionPosition = SPECIMEN_SCORE_SLIDE_RETRACTION;
                        wristPitchPos = SPECIMEN_SCORE_WRIST_PITCH;
                        armState = STATE_IDLE;
                    }
                    break;
            }

            // Final wrist update
            wristPitch.setPosition(wristPitchPos);

            // Telemetry
            telemetry.addData("Drive FL", fl);
            telemetry.addData("Drive FR", fr);
            telemetry.addData("Drive BL", bl);
            telemetry.addData("Drive BR", br);

            telemetry.addData("ArmState", armState);
            telemetry.addData("SlidePitch Pos", slidePitch.getCurrentPosition());
            telemetry.addData("SlideRetraction Pos", slideRetraction.getCurrentPosition());
            telemetry.addData("Target Pitch", targetSlidePitchPosition);
            telemetry.addData("Target Retract", targetSlideRetractionPosition);
            telemetry.addData("Wrist Pitch", wristPitchPos);
            telemetry.addData("Slide Pitch Power", slidePitch.getPower());
            telemetry.addData("Slide Retraction Power", slideRetraction.getPower());
            telemetry.update();
        }
    }
}
