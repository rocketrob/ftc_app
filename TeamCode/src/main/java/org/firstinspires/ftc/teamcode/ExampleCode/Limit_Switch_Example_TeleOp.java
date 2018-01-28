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
package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file contains an example of a setting upper/lower limits for running a motor.
 *
 */

@TeleOp(name="Example: MotorLimit", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class Limit_Switch_Example_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //motors

    DcMotor motorArm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

         motorArm = hardwareMap.dcMotor.get("motorArm");

        // eg: Set the drive motor directions:
         motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        // reset Encoder to zero
         motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // runs motor faster than when set to RUN_USING_ENCODER
         //motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // not sure why this happens

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoder", motorArm.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            // Display running time and Encoder value
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Encoder Clicks", + motorArm.getCurrentPosition());
            telemetry.update();

            // Arm Control - Uses dual buttons to control motor direction.
            // Uses Encoder values to set upper and lower limits to protect motors from over-driving lift
//
            if (gamepad1.right_bumper && motorArm.getCurrentPosition() > 0.0) //bumper pressed AND encoder greater that lower limit
            {
                motorArm.setPower(-gamepad1.right_trigger / 2.0); // let trigger run -motor
            }
//
            else if (!gamepad1.right_bumper && motorArm.getCurrentPosition() < 10000.0) //bumper NOT pressed AND encoder less than Max limit
            {
                motorArm.setPower(gamepad1.right_trigger / 2.0); //let trigger run +motor
            }

            else
            {
                motorArm.setPower(0.0); // else not trigger, then set to off or some value of 'hold' power
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
