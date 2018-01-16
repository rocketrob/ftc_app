/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Testing Stetson's sweeper button toggle program from Dan
 */


@TeleOp(name="Toggle2", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class ToggleButtonServo extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor sweeper = null;

    //Servo
    Servo servoHandTopRight    = null;
    Servo servoHandTopLeft    = null;


    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorLeft = hardwareMap.dcMotor.get("motorL");
        motorRight = hardwareMap.dcMotor.get("motorR");
        sweeper = hardwareMap.dcMotor.get("sweeper");

        servoHandTopLeft = hardwareMap.servo.get("servoHandTopLeft");
        servoHandTopRight = hardwareMap.servo.get("servoHandTopRight");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        sweeper.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        // Declare some variables
        double motorDirection = -1.0;   //Keeps track of the direction for the sweeper motor
        boolean yPrevState = false;      //Keeps track of whether the button was previously pressed or not so we know when it is released
        boolean yCurrState = false;
        boolean xPrevState = false;      //Keeps track of whether the button was previously pressed or not so we know when it is released
        boolean xCurrState = false;
        int padA_count = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(gamepad1.right_stick_y);


//Toggle open when button 'y' pressed and closed when pressed again.

            // set current state to that of the button
            yCurrState = gamepad1.y;


            if ((yCurrState == false) ) {
                if(gamepad1.y) {
                    padA_count = +1; //
                    yCurrState = false;
                    //open
                    servoHandTopRight.setPosition(0.2);
                    servoHandTopLeft.setPosition(0.8);
                }
            }
            else{
                if(gamepad1.y){
                    padA_count =+1;
                    yCurrState = true;
                    //close
                    servoHandTopRight.setPosition(0.85);
                    servoHandTopLeft.setPosition(0.15);
                }
            }


//            //boolean declared before init()
////            int gamepad1a_count = 0;
///             declared in init()
//              boolean directionState = false;

//            if(directionState == false){
//                if(gamepad1.x){
//                    gamepad1a_count += 1;
//                    directionState = true;
//
//                    gripperRotation.setPosition(0.435);
//                }
//            }else{
//                if(gamepad2.dpad_right){
//                    dpad_right_count += 1;
//                    directionState = false;
//
//                    gripperRotation.setPosition(0.915);
//                }
//            } //declared in loop
//            telemetry.addData("Direction State:", directionState);
//            telemetry.addData("dpad_right_count:", count);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }
}