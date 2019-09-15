//IMPORTANT - THIS IS ALL COMMENTED CODE FOR THE ARM'S TESTING

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="TeleOp_Arm", group="Linear Opmode")
// @Disabled
public class TeleOp_Arm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean onPos = true; 
    private double pos = 0; 
    private DcMotor Grabber;
    private DcMotor Pivot;
    private CRServo Extender = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28; 
    static final double     CHAIN_GEAR_REDUCTION    = 40/15.0;
    static final double     DRIVE_GEAR_REDUCTION    = 40;
    static final double     DEGREES_PER_REV         = 360;
    static final double     TICK_PER_DEGREE        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * CHAIN_GEAR_REDUCTION)/(DEGREES_PER_REV);
    static final double     TURN_SPEED              = 0.2;
    static final double     GRAB_SPEED              = 0.3;
    static final double     PIVOT_SPEED             = 0.2;


    @Override
    public void runOpMode() {

        


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pivot = hardwareMap.get(DcMotor.class, "Pivot");

        Grabber = hardwareMap.get(DcMotor.class, "Grabber");
        Extender = hardwareMap.get(CRServo.class, "Extender");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Reset Encoders and set mode to BRAKE
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)





        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            

            telemetry.addData("Power: ", gamepad2.right_trigger);
            telemetry.update();

            //Setting mode run to position

             Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);



             //When the right bumper is pressed, move the servo forward (clockwise)

   
            while(gamepad2.a){
                Grabber.setPower(GRAB_SPEED);

             if(gamepad2.right_bumper){
                Extender.setPower(1);
             }else if(gamepad2.left_bumper){ // When the left bumper is pressed, move the servo backwards
                Extender.setPower(0);
             }
             telemetry.addData("Grabbing Minerals", " ");
             telemetry.update();
            }


            while(!gamepad2.a){
                Grabber.setPower(0);

             if(gamepad2.right_bumper){
                Extender.setPower(1);
                telemetry.addData("Extending", "Full Power");
             }else if(gamepad2.left_bumper){ // When the left bumper is pressed, move the servo backwards
                Extender.setPower(0);
                telemetry.addData("Retracting", "Full Power");
                telemetry.update();
             }   if(gamepad2.y){
                //Moves Pivot Motor Clockwise
                onPos = false; 
                pos = ((int) (TICK_PER_DEGREE * 158));


            }else if(gamepad2.x){
                //Moves Pivot Motor Counter-Clockwise (back to 0)
                onPos = false; 
                pos = 0; 
            }else if(gamepad2.b) {
                onPos = false;
                pos = ((int) (TICK_PER_DEGREE * 64));
            }




            // Rotate the arm code (everything above is setting new variables and changing the boolean value)
            rotateArm(pos);
            }


            
          

                         // Show the elapsed game time and wheel power.

            telemetry.addData("Current Position", Pivot.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            
            telemetry.update();

            }
            

        

          
        }


        //Code to rotate arm
        public void rotateArm(double targetPos)
        {



        	//If the arm isn't at the position it should be, run the following - 
            if(!onPos)
            {
                Pivot.setTargetPosition((int)targetPos * -1);
                Pivot.setPower(0.05);
                if(Pivot.getCurrentPosition()==targetPos)
                    onPos = true;
            }else 
            {
                Pivot.setPower(0);
            }

        }
    }



