

package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="OffSeason_23333_V2_Auto")
@Config

public class OffSeason_23333_V2_Auto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices.
    DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
    DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
    DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
    DcMotor backRight = hardwareMap.dcMotor.get("backRight");

    DcMotor slidePitch = hardwareMap.dcMotor.get("slidePitch");
    DcMotor slideRetraction = hardwareMap.dcMotor.get("slideRetraction");

    CRServo clawRoll = hardwareMap.crservo.get("clawRoll");
    CRServo clawGrip = hardwareMap.crservo.get("clawGrip");

    Servo wristPitch = hardwareMap.servo.get("wristPitch");

    @Override
    public void runOpMode() {


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Main loop here.

            telemetry.addData("Status", "Running Autonomous");
            telemetry.update();
        }
    }}
