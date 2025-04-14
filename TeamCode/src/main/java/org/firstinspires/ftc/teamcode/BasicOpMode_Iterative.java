package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {

    private double Kp = 0.05;
    private double Ki = 0.001;
    private double Kd = 0.01;

    private double wristPitchPosition = 0.5;
    private boolean isManualWristPitchControl = false;

    private double previousError = 0;
    private double integral = 0;
    private double targetPosition = 0;
    private double manualWristOffset = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");
        Servo clawRoll = hardwareMap.servo.get("clawRoll");
        Servo clawGrip = hardwareMap.servo.get("clawGrip");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Drive control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // Slide pitch control
            if (gamepad1.y) {
                targetPosition += 1;
            } else if (gamepad1.x) {
                targetPosition -= 1;
            }

            // Slide retraction control
            if (gamepad1.right_trigger > 0) {
                slideRetraction.setPower(0.8);
            } else if (gamepad1.left_trigger > 0) {
                slideRetraction.setPower(-0.8);
            } else {
                slideRetraction.setPower(0);
            }

            // Wrist pitch control
            if (gamepad1.dpad_up) {
                manualWristOffset += 0.01;
                isManualWristPitchControl = true;
            } else if (gamepad1.dpad_down) {
                manualWristOffset -= 0.01;
                isManualWristPitchControl = true;
            }

            double baseWristPitch = 0.5 - slidePitch.getCurrentPosition() / 1000.0;
            wristPitchPosition = baseWristPitch + manualWristOffset;
            wristPitchPosition = Math.max(0, Math.min(1, wristPitchPosition));
            wristPitch.setPosition(wristPitchPosition);

            // Claw roll
            if (gamepad1.dpad_left) {
                clawRoll.setPosition(clawRoll.getPosition() + 0.05);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPosition(clawRoll.getPosition() - 0.05);
            }

            // Claw grip
            if (gamepad1.right_bumper) {
                clawGrip.setPosition(180);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPosition(90);
            }

            // PID control for slide pitch
            double error = targetPosition - slidePitch.getCurrentPosition();
            integral += error;
            double derivative = error - previousError;
            double pidOutput = Kp * error + Ki * integral + Kd * derivative;

            slidePitch.setPower(pidOutput);
            previousError = error;

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Wrist Pitch", wristPitchPosition);
            telemetry.addData("Manual Wrist Offset", manualWristOffset);
            telemetry.addData("Claw Roll", clawRoll.getPosition());
            telemetry.addData("Claw Grip", clawGrip.getPosition());
            telemetry.addData("Slide Pitch Power", slidePitch.getPower());
            telemetry.addData("PID Kp", Kp);
            telemetry.addData("PID Ki", Ki);
            telemetry.addData("PID Kd", Kd);
            telemetry.addData("Slide Pitch Position", slidePitch.getCurrentPosition());
            telemetry.update();
        }
    }
}
