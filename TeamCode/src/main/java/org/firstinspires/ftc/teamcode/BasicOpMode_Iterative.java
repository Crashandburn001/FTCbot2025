package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {

    private double Kp = 0.05;
    private double Ki = 0.001;
    private double Kd = 0.01;

    private File pidFile = new File(hardwareMap.appContext.getFilesDir(), "pid_config.txt");

    // Variable to store the current wrist pitch position
    private double wristPitchPosition = 0.5;

    // Variable to track if the wrist pitch is being manually controlled
    private boolean isManualWristPitchControl = false;

    // PID variables for slide pitch
    private double previousError = 0;
    private double integral = 0;
    private double targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Load PID values from file at the start
        loadPIDValues();

        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
        DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

        Servo wristPitch = hardwareMap.servo.get("wristPitch");
        Servo clawRoll = hardwareMap.servo.get("clawRoll");
        Servo clawGrip = hardwareMap.servo.get("clawGrip");

        // Reverse the right side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when not moving slidePitch
        slidePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Drive control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Slide pitch control with Triangle (Y) and X
            if (gamepad1.y) {
                targetPosition = targetPosition + 1; // Move up (geared)
            } else if (gamepad1.x) {
                targetPosition = targetPosition - 1; // Move down (geared)
            } else {
                slidePitch.setPower(0); // Stop
            }

            // Slide retraction control with R2 (extend) and L2 (retract)
            if (gamepad1.right_trigger > 0) {
                slideRetraction.setPower(0.8); // Extend (not geared)
            } else if (gamepad1.left_trigger > 0) {
                slideRetraction.setPower(-0.8); // Retract (not geared)
            } else {
                slideRetraction.setPower(0); // Stop
            }

            // Adjust wrist pitch automatically based on slide pitch
            double slidePitchPower = slidePitch.getPower();
            if (slidePitchPower != 0) {
                // Moving slide up or down, adjust wrist pitch to maintain orientation
                if (!isManualWristPitchControl) {
                    wristPitchPosition = 0.5 - slidePitch.getCurrentPosition() / 1000.0; // Adjust formula for your system's needs
                }
            }

            // Manual control of wrist pitch
            if (gamepad1.dpad_up) {
                wristPitchPosition += 0.05;  // Move wrist pitch up
                isManualWristPitchControl = true; // Start manual control
            } else if (gamepad1.dpad_down) {
                wristPitchPosition -= 0.05;  // Move wrist pitch down
                isManualWristPitchControl = true; // Start manual control
            } else {
                isManualWristPitchControl = false; // No manual control
            }

            // Clamp wrist pitch position to valid range
            wristPitchPosition = Math.max(0, Math.min(1, wristPitchPosition));

            wristPitch.setPosition(wristPitchPosition);

            // Claw Yaw/Roll using gamepad left and right
            if (gamepad1.dpad_left) {
                clawRoll.setPosition(clawRoll.getPosition() + 0.05);
            } else if (gamepad1.dpad_right) {
                clawRoll.setPosition(clawRoll.getPosition() - 0.05);
            }

            // Claw Grip control
            if (gamepad1.right_bumper) {
                clawGrip.setPosition(180);
            } else if (gamepad1.left_bumper) {
                clawGrip.setPosition(90);
            }

            // Update PID control for slide pitch (with simple proportional control for now)
            double error = targetPosition - slidePitch.getCurrentPosition();
            integral += error;
            double derivative = error - previousError;
            double pidOutput = Kp * error + Ki * integral + Kd * derivative;

            slidePitch.setPower(pidOutput); // Apply the PID output to the slide pitch motor

            // Save PID values if buttons on gamepad 2 are pressed
            if (gamepad2.dpad_up) {
                Kp += 0.01;
                savePIDValues();
            } else if (gamepad2.dpad_down) {
                Kp -= 0.01;
                savePIDValues();
            }

            if (gamepad2.dpad_left) {
                Ki += 0.0001;
                savePIDValues();
            } else if (gamepad2.dpad_right) {
                Ki -= 0.0001;
                savePIDValues();
            }

            if (gamepad2.left_bumper) {
                Kd += 0.001;
                savePIDValues();
            } else if (gamepad2.right_bumper) {
                Kd -= 0.001;
                savePIDValues();
            }

            // Telemetry
            telemetry.addData("Status:", "Running");
            telemetry.addData("Wrist Pitch:", wristPitch.getPosition());
            telemetry.addData("Claw Roll:", clawRoll.getPosition());
            telemetry.addData("Claw Grip:", clawGrip.getPosition());
            telemetry.addData("Slide Pitch Power:", slidePitch.getPower());
            telemetry.addData("PID Kp:", Kp);
            telemetry.addData("PID Ki:", Ki);
            telemetry.addData("PID Kd:", Kd);
            telemetry.addData("Slide Pitch Position:", slidePitch.getCurrentPosition());
        }
    }

    // Load PID values from the file
    private void loadPIDValues() {
        if (pidFile.exists()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(pidFile))) {
                Kp = Double.parseDouble(reader.readLine());
                Ki = Double.parseDouble(reader.readLine());
                Kd = Double.parseDouble(reader.readLine());
            } catch (IOException e) {
                telemetry.addData("Error", "Failed to load PID values");
            }
        }
    }

    // Save PID values to file
    private void savePIDValues() {
        try (FileWriter writer = new FileWriter(pidFile)) {
            writer.write(Kp + "\n");
            writer.write(Ki + "\n");
            writer.write(Kd + "\n");
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save PID values");
        }
    }
}
