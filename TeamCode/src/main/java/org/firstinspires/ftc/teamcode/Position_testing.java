package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Position_testing")
public class Position_testing extends LinearOpMode {

    // Dashboard-editable target positions (override gamepad targets)
    public static int dashSlidePitchPos = 0;
    public static int dashSlideRetractionPos = 0;
    public static double dashWristPitchPos = 0.5;
    public static double dashClawGripPos = 0.5;

    // Increment steps for gamepad control
    public static int slidePitchIncrement = 20;
    public static int slideRetractionIncrement = 10;
    public static double wristPitchIncrement = 0.05;
    public static double clawGripIncrement = 0.05;

    // Internal targets
    private int targetSlidePitchPosition = 0;
    private int targetSlideRetractionPosition = 0;
    private double targetWristPitchPosition = 0.5;
    private double targetClawGripPosition = 0.5;

    @Override
    public void runOpMode() {

        // Initialize hardware with null checks
        DcMotor slidePitch = null;
        if (hardwareMap.dcMotor.contains("slidePitch")) {
            slidePitch = hardwareMap.dcMotor.get("slidePitch");
            slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidePitch.setDirection(DcMotor.Direction.REVERSE);
            slidePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidePitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        DcMotor slideRetraction = null;
        if (hardwareMap.dcMotor.contains("slideRetraction")) {
            slideRetraction = hardwareMap.dcMotor.get("slideRetraction");
            slideRetraction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRetraction.setDirection(DcMotor.Direction.FORWARD);
            slideRetraction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRetraction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        Servo wristPitch = null;
        if (hardwareMap.servo.contains("wristPitch")) {
            wristPitch = hardwareMap.servo.get("wristPitch");
        }

        Servo clawGrip = null;
        if (hardwareMap.servo.contains("clawGrip")) {
            clawGrip = hardwareMap.servo.get("clawGrip");
        }

        waitForStart();

        if (isStopRequested()) return;

        // Initialize targets to current positions (if hardware exists)
        if (slidePitch != null) targetSlidePitchPosition = slidePitch.getCurrentPosition();
        if (slideRetraction != null) targetSlideRetractionPosition = slideRetraction.getCurrentPosition();
        if (wristPitch != null) targetWristPitchPosition = wristPitch.getPosition();
        if (clawGrip != null) targetClawGripPosition = clawGrip.getPosition();

        while (opModeIsActive() && !isStopRequested()) {

            // Incremental adjustments with gamepad1
            if (gamepad1.y) {
                targetSlidePitchPosition += slidePitchIncrement;
            } else if (gamepad1.x) {
                targetSlidePitchPosition -= slidePitchIncrement;
            }

            if (gamepad1.right_trigger > 0.1) {
                targetSlideRetractionPosition += slideRetractionIncrement;
            } else if (gamepad1.left_trigger > 0.1) {
                targetSlideRetractionPosition -= slideRetractionIncrement;
            }

            if (gamepad1.dpad_up) {
                targetWristPitchPosition = clamp(targetWristPitchPosition + wristPitchIncrement, 0, 1);
            } else if (gamepad1.dpad_down) {
                targetWristPitchPosition = clamp(targetWristPitchPosition - wristPitchIncrement, 0, 1);
            }

            if (gamepad1.dpad_right) {
                targetClawGripPosition = clamp(targetClawGripPosition + clawGripIncrement, 0, 1);
            } else if (gamepad1.dpad_left) {
                targetClawGripPosition = clamp(targetClawGripPosition - clawGripIncrement, 0, 1);
            }

            // Override targets with dashboard values if changed
            targetSlidePitchPosition = dashSlidePitchPos;
            targetSlideRetractionPosition = dashSlideRetractionPos;
            targetWristPitchPosition = clamp(dashWristPitchPos, 0, 1);
            targetClawGripPosition = clamp(dashClawGripPos, 0, 1);

            // Apply targets to hardware if present
            if (slidePitch != null) {
                slidePitch.setTargetPosition(targetSlidePitchPosition);
                slidePitch.setPower(1);
            }

            if (slideRetraction != null) {
                // Clamp slideRetraction to limits (example limits -1920 to 40)
                targetSlideRetractionPosition = Math.min(targetSlideRetractionPosition, 40);
                targetSlideRetractionPosition = Math.max(targetSlideRetractionPosition, -1920);
                slideRetraction.setTargetPosition(targetSlideRetractionPosition);
                slideRetraction.setPower(1);
            }

            if (wristPitch != null) {
                wristPitch.setPosition(targetWristPitchPosition);
            }

            if (clawGrip != null) {
                clawGrip.setPosition(targetClawGripPosition);
            }

            // Telemetry to FTC Dashboard
            telemetry.addData("SlidePitch Target", targetSlidePitchPosition);
            telemetry.addData("SlidePitch Current", slidePitch != null ? slidePitch.getCurrentPosition() : "N/A");

            telemetry.addData("SlideRetraction Target", targetSlideRetractionPosition);
            telemetry.addData("SlideRetraction Current", slideRetraction != null ? slideRetraction.getCurrentPosition() : "N/A");

            telemetry.addData("WristPitch Target", targetWristPitchPosition);
            telemetry.addData("WristPitch Actual", wristPitch != null ? wristPitch.getPosition() : "N/A");

            telemetry.addData("ClawGrip Target", targetClawGripPosition);
            telemetry.addData("ClawGrip Actual", clawGrip != null ? clawGrip.getPosition() : "N/A");

            telemetry.update();

            sleep(20);
        }
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
