
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the TeleOp period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo claw = null;
    private Servo yaw = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive" );
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive"  );
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive" );
        arm        = hardwareMap.get(DcMotor.class, "arm"              );

        claw       = hardwareMap.get(Servo  .class, "claw"             );
        yaw        = hardwareMap.get(Servo  .class, "yaw"              );

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront .setDirection(DcMotor.Direction.FORWARD);
        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        arm       .setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower  ;
        double leftBackPower   ;
        double rightFrontPower ;
        double rightBackPower  ;
        double armPower        ;

        double drive      = - gamepad1.left_stick_y                          ;
        double turn       =   gamepad1.right_stick_x                         ;
        // double slideup    =   gamepad1.right_trigger - gamepad1.left_trigger ;
        double slideangle =   gamepad1.right_stick_y                         ;

        leftFrontPower    =   Range.clip(drive + turn, -1.0, 1.0) ;
        leftBackPower     =   Range.clip(drive + turn, -1.0, 1.0) ;
        rightFrontPower   =   Range.clip(drive - turn, -1.0, 1.0) ;
        rightBackPower    =   Range.clip(drive - turn, -1.0, 1.0) ;
        armPower          =   Range.clip(slideangle  , -1.0, 1.0) ;

        if(gamepad1.left_bumper){
            claw.setPosition(1);
        } else if (gamepad1.right_bumper){
            claw.setPosition(0);
        }

        if(gamepad1.dpad_left){
            yaw.setPosition(1);
        } else if (gamepad1.dpad_right){
            yaw.setPosition(0);
        }

        // Send calculated power to wheels
        leftFront .setPower(leftFrontPower) ;
        leftBack  .setPower(leftBackPower)  ;
        rightFront.setPower(rightFrontPower);
        rightBack .setPower(rightBackPower) ;
        arm       .setPower(armPower)       ;


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
