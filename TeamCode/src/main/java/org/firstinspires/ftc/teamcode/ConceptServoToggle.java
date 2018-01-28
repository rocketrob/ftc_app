package org.firstinspires.ftc.teamcode;

/**
 * Created by hsrobotics on 1/23/2018.
 */

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Testing 4 servo grabber
 * with added toggle buttons to run top/bottom servos separately
 */
@TeleOp(name="MrR TeleopPOV Toggle", group="Concept")
//@Disabled

public class ConceptServoToggle extends LinearOpMode {
    /* Declare Hardware */

    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;

    //Servo
    Servo servoHandTopRight    = null;
    Servo servoHandTopLeft    = null;
    Servo servoHandBottomRight    = null;
    Servo servoHandBottomLeft    = null;
    double LEFT_SERVO_CLOSED = 0.2;  // servo values
    double LEFT_SERVO_OPEN = 0.8;
    double RIGHT_SERVO_CLOSED = 0.8;
    double RIGHT_SERVO_OPEN = 0.2;

    @Override
    public void runOpMode() {

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorLeft = hardwareMap.dcMotor.get("motorL");
        motorRight = hardwareMap.dcMotor.get("motorR");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        servoHandTopLeft = hardwareMap.servo.get("servoHandTopLeft");
        servoHandTopRight = hardwareMap.servo.get("servoHandTopRight");
        servoHandBottomLeft = hardwareMap.servo.get("servoHandBottomLeft");
        servoHandBottomRight = hardwareMap.servo.get("servoHandBottomRight");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", "Waiting for Start..."); //
        telemetry.update();

        // establish variables to monitor buttons and servo conditions
        boolean xPressed = false;
        boolean yPressed = false;
        boolean topOpen = false;
        boolean bottomOpen = false;


// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


// Main servo controls to open and close all 4 servos together
// using pushbot configuration for mirrored servos

            if (gamepad2.a) {
// Set all servos to open position
                servoHandTopLeft.setPosition(LEFT_SERVO_OPEN);
                servoHandBottomLeft.setPosition(LEFT_SERVO_OPEN);
                servoHandTopRight.setPosition(RIGHT_SERVO_OPEN);
                servoHandBottomRight.setPosition(RIGHT_SERVO_OPEN);

                topOpen = true; // updates current position of servo
                bottomOpen = true;
            } else if (gamepad2.b) {
// Set all servos to closed position
                servoHandTopLeft.setPosition(LEFT_SERVO_CLOSED);
                servoHandBottomLeft.setPosition(LEFT_SERVO_CLOSED);
                servoHandTopRight.setPosition(RIGHT_SERVO_CLOSED);
                servoHandBottomRight.setPosition(RIGHT_SERVO_CLOSED);

                topOpen = false; // updates current position of servo
                bottomOpen = false;
            } else {
                if (gamepad2.x && !xPressed) { // if x is now down and wasn't previously
// Toggle top servos only
                    if (topOpen) { // if was topOpen, then close
                        servoHandTopLeft.setPosition(LEFT_SERVO_CLOSED);
                        servoHandTopRight.setPosition(RIGHT_SERVO_CLOSED);

                        topOpen = false; // toggle topOpen to false
                    } else { // else if not topOpen, then open
                        servoHandTopLeft.setPosition(LEFT_SERVO_OPEN);
                        servoHandTopRight.setPosition(RIGHT_SERVO_OPEN);

                        topOpen = true;// toggle topOpen to true
                    }
                }

                if (gamepad2.y && !yPressed) {
// Toggle bottom servos only
                    if (bottomOpen) {
                        servoHandBottomLeft.setPosition(LEFT_SERVO_CLOSED);
                        servoHandBottomRight.setPosition(RIGHT_SERVO_CLOSED);

                        bottomOpen = false;
                    } else {
                        servoHandBottomLeft.setPosition(LEFT_SERVO_OPEN);
                        servoHandBottomRight.setPosition(RIGHT_SERVO_OPEN);

                        bottomOpen = true;
                    }
                }
            }

            xPressed = gamepad2.x; // resets so next time thru it knows current state of buttons
            yPressed = gamepad2.y;

//display the button pressed
            telemetry.addData("Claw", "A " + String.valueOf(gamepad2.a));
            telemetry.addData("Single Claw", "x " + String.valueOf(gamepad2.x ));
//telemetry.addData("newPos: ", newY_Pos);
            telemetry.update();

            idle();

        }//While OpMode Active
    }//run opMode
}//Linear OpMode