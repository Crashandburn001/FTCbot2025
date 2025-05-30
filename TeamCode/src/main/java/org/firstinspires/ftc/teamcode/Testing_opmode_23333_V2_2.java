package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@Config
@TeleOp(name = "Position_testing")
public class Testing_opmode_23333_V2_2 extends LinearOpMode {

    public static int targetSlidePitchPosition = 0;
    public static int targetSlideRetractionPosition = 0;

    public static double wristPitchPos = 0.5;
    public static float clawRollPower = 0;

    public static int adjustmentsSlidePitch = 20;
    public static double adjustmentsWristPitch = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");
        CRServo clawRoll = hardwareMap.crservo.get("clawRoll");
        CRServo clawGrip = hardwareMap.crservo.get("clawGrip");

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

            // Slide pitch control (gamepad1)
            if (gamepad1.y) {
                targetSlidePitchPosition += adjustmentsSlidePitch;
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= adjustmentsSlidePitch;
            }
            slidePitch.setTargetPosition(targetSlidePitchPosition);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePitch.setPower(1);

            // Slide retraction control (gamepad1)
            if (gamepad1.right_trigger > 0.1) {
                targetSlideRetractionPosition += 10;
            } else if (gamepad1.left_trigger > 0.1) {
                targetSlideRetractionPosition -= 10;
            }

            // Clamp
            targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, 40);
            targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, -1920);

            slideRetraction.setTargetPosition(targetSlideRetractionPosition);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRetraction.setPower(1);

            // Wrist pitch control
            if (gamepad1.dpad_up) {
                wristPitchPos += adjustmentsWristPitch;
            } else if (gamepad1.dpad_down) {
                wristPitchPos -= adjustmentsWristPitch;
            }
            wristPitchPos = Math.max(0, Math.min(1, wristPitchPos));
            wristPitch.setPosition(wristPitchPos);

            // Claw roll control (original)
            if (gamepad1.dpad_left) {
                clawRoll.setPower(0.5f);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPower(-0.5f);
            } else {
                clawRoll.setPower(0);
            }

            // Claw grip (CR servo, same control)
            if (gamepad1.right_bumper) {
                clawGrip.setPower(0.8);
                gamepad1.rumble(1);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPower(-0.8);
            } else {
                clawGrip.setPower(0);
                gamepad1.stopRumble();
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
