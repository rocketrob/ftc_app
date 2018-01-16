package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ExampleCode.MyBotHardwareSetup;


/**
 * Created by hsrobotics on 1/16/2018.
 * Concept Using Motor Encoder to set Arm Drive motor upper/lower limits and monitor and hold position
 */

@TeleOp(name="Example: Concept HoldArm", group="Examples")  // @Autonomous(...) is the other common choice
//@Disabled
public class ConceptHoldArm extends LinearOpMode{

    ConceptBotHardwareSetup bot = new ConceptBotHardwareSetup(); //set up remote to robot hardware configuration
    //variable for arm limits and hold position
    double  armMinPos        = 0.0;      // encoder position for arm at bottom
    double  armMaxPos        = 5380.0;   // encoder position for arm at top
    int     armHoldPosition;             // reading of arm position when buttons released to hold
    double  slopeVal         = 2000.0;   // increase or decrease to perfect

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * Constructor
     */
    public ConceptHoldArm() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup
        //initialze
        armHoldPosition = bot.motorArm.getCurrentPosition();
        
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.addData("armPostion: ", + bot.motorArm.getCurrentPosition());
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
    
    /************************
     * TeleOp Code Below://
     *************************/
           
        // Display running time and Encoder value
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("ArmPosition: ", + bot.motorArm.getCurrentPosition());
        telemetry.update();

        // Arm Control - Uses left/right triggers to control motor direction.
        // Uses Encoder values to set upper and lower limits to protect motors from over-driving lift
        // and to hold arm position on decent to account for gravitational inertia

        if (gamepad2.left_trigger > 0.0 && bot.motorArm.getCurrentPosition() > armMinPos) // encoder greater that lower limit
        {
            bot.motorArm.setPower(-gamepad2.left_trigger / 2.0); // let trigger run -motor DOWN / div in half to slow motor
            armHoldPosition = bot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
        }
        else if (gamepad2.right_trigger > 0.0 && bot.motorArm.getCurrentPosition() < armMaxPos) //encoder less than Max limit
        {
            bot.motorArm.setPower(gamepad2.right_trigger); //let trigger run +motor UP
            armHoldPosition = bot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
        }
        else //triggers are released - try to maintain the current position
        {
            bot.motorArm.setPower((double) (armHoldPosition - bot.motorArm.getCurrentPosition()) / slopeVal);   // Note that if the lift is lower than desired position,
                                                                                                                // the subtraction will be positive and the motor will
                                                                                                                // attempt to raise the lift. If it is too high it will
                                                                                                                // be negative and thus try to lower the lift
                                                                                                                // adjust slopeVal to acheived perfect hold power
        }

        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

    }//runOpMode
}//ConceptHoldArm
