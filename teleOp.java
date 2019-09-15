/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="TeleOp", group="Linear Opmode")
//@Disabled
public class teleOp extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public boolean onPos = true;
    public double pos = 0;
    public DcMotor Pivot;
    public CRServo Extender;
    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor lBackMotor;
    public DcMotor rBackMotor;
    public DcMotor Grabber;
    public DcMotor Xmotor;
    public Servo tmServo;

    static final double COUNTS_PER_MOTOR_REV_latch = 24;
    static final double DRIVE_GEAR_REDUCTION_latch = 60.0;
    static final double WHEEL_DIAMETER_INCHES_latch = 1.25;
    static final double COUNTS_PER_INCH_latch = (COUNTS_PER_MOTOR_REV_latch * DRIVE_GEAR_REDUCTION_latch) / (Math.PI * WHEEL_DIAMETER_INCHES_latch);
                                                                //calcutes the encoder tics for the latch
    static final double MOVE_SPEED_latch = 0.1;                 //sets the move speed for the latch to 0.1

    static final double     COUNTS_PER_MOTOR_REV    = 28;       //amount of motor revs
    static final double     CHAIN_GEAR_REDUCTION    = 40/15.0;  //the amount of the chain gear reduction
    static final double     DRIVE_GEAR_REDUCTION    = 40;       // The amount of drive gear reduction
    static final double     DEGREES_PER_REV         = 360;      // How many degrees in 1 revolution
    static final double     TICK_PER_DEGREE        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * CHAIN_GEAR_REDUCTION)/(DEGREES_PER_REV);
                                                                //calculates the ticks per degree for the pivot
    static final double     TURN_SPEED              = 0.2;      //the turn speed of our robot
    static final double     GRAB_SPEED              = 0.4;      // The grabber speed
    static final double     GRAB_SPEED_OP           = -0.2;     // the reverse grabber speed
    static final double     PIVOT_SPEED             = 0.2;      // the pivot speed


    int ARM_STATE = 1;                                          // sets the arm state variable


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Xmotor = hardwareMap.get(DcMotor.class, "Xmotor");
                                                                // Lift mechanism
        Grabber = hardwareMap.get(DcMotor.class, "Grabber");
                                                                // Entraption star mechanism
        Pivot = hardwareMap.get(DcMotor.class, "Pivot");
                                                                // Pivot mechanism
        tmServo = hardwareMap.get(Servo.class, "TMServo");
                                                                 // Team marker servo mechanism
        Extender = hardwareMap.get(CRServo.class, "Extender");
                                                                 // Extender mechanism
        lFrontMotor = hardwareMap.get(DcMotor.class, "left_Front_Motor");
                                                                 //left front motor
        rFrontMotor = hardwareMap.get(DcMotor.class, "right_Front_Motor");
                                                                 // right front motor
        lBackMotor = hardwareMap.get(DcMotor.class, "left_Back_Motor");
                                                                 // left back motor
        rBackMotor = hardwareMap.get(DcMotor.class, "right_Back_Motor");
                                                                 // right back motor



        //Reset Encoders and set mode to BRAKE
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();  // Wait for the game to start (driver presses PLAY)
        runtime.reset();

        //                                                          run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            Setting original power to 0.8 out of 1

            lFrontMotor.setPower(-gamepad1.left_stick_y * 0.8);
            lBackMotor.setPower(-gamepad1.left_stick_y * 0.8);
            rFrontMotor.setPower(-gamepad1.right_stick_y * 0.8);
            rBackMotor.setPower(-gamepad1.right_stick_y * 0.8);

//                                                                  Setting Right Motors to reverse (so we can drive straight)

            rBackMotor.setDirection(DcMotor.Direction.REVERSE);
            rFrontMotor.setDirection(DcMotor.Direction.REVERSE);

            //Setting mode run to position

            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                                                                   Code for Entraption stars; when the bumpers on gamepad 2 are pressed, turn the stars either clockwise or counter-clockwise

            if(gamepad2.right_bumper){
                Grabber.setPower(GRAB_SPEED);
            } else if(gamepad2.left_bumper) {
                Grabber.setPower(-GRAB_SPEED);
            } else {
                Grabber.setPower(0);
            }
//                                                              Code to bring the latch up to the latching position, 7 (after the aiming 6-inch position)
            if (gamepad2.dpad_up) {
                Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("dpad_up:", "on");
                telemetry.update();
                Xmotor.setTargetPosition((int) (7.75 * COUNTS_PER_INCH_latch));
                Xmotor.setPower(MOVE_SPEED_latch);
                Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
//                                                               Brings the latch to the aiming position, to create a spot to latch under the lander
            if (gamepad2.dpad_left) {
                Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("left:", "on");
                telemetry.update();
                Xmotor.setTargetPosition((int) (6 * COUNTS_PER_INCH_latch));
                Xmotor.setPower(MOVE_SPEED_latch);
                Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

//                                                              Brings the robot off of the ground, supported solely and completely by the lander

            if (gamepad2.dpad_down) {
                Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // encoderMove(MOVE_SPEED, -7.25, 10.00);
                telemetry.addData("down:", "on");
                telemetry.update();
                Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Xmotor.setTargetPosition((int) (4 * COUNTS_PER_INCH_latch));
                Xmotor.setPower(0.3);

            }

//                                                              Brings the latch to it's lowest point, primarily used in testing phases
            if (gamepad2.dpad_right) {
                Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                                              //makes the lift motor use run using run to position
                telemetry.addData("right:", "on");
                telemetry.update();
                Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                                              //sets the lift behaviour of the lift to brake
                Xmotor.setTargetPosition(0);                  //sets the taget position of the lift to the hung position
                Xmotor.setPower(MOVE_SPEED_latch);            //sets the movespeed of the latch to the variable
            }

                                                                 //Resets the Servo after

            if (gamepad1.right_bumper == true) {                //If the right bumper is pressed, then set the team marker servo to 0
                tmServo.setPosition(0);                         //bring up the team marker servo
            }

            if(gamepad1.left_bumper){                           //If the left bumper is pressed, set the power to 0.3
                lFrontMotor.setPower(-gamepad1.left_stick_y * 0.3);
                                                                //Set the left front motor power to 0.3
                lBackMotor.setPower(-gamepad1.left_stick_y * 0.3);
                                                                //Set the left back motor power to 0.3
                rFrontMotor.setPower(-gamepad1.right_stick_y * 0.3);
                                                                //Set the right front motor power to 0.3
                rBackMotor.setPower(-gamepad1.right_stick_y * 0.3);
                                                                //Set the right back motor power to 0.3
                telemetry.addData("Speed:", "0.3");
                telemetry.update();


            }


            switch(ARM_STATE) {                                 // starting the switch for the state machine

                case 1:                                         // Initialization (state to 1)
                    onPos = false;                              // tap the button and not hold it
                    pos = ((int) (TICK_PER_DEGREE * 77));       // set pivot to travel state
                    ARM_STATE = 2;                              // arm state 2
                    break;

                case 2:                                         // Travel Pos to Crater
                    if(gamepad2.a) {
                        onPos = false;                         // tap the button and not hold it
                        pos = ((int) (TICK_PER_DEGREE * 90));  // set pivot to complete horizontal state
                        ARM_STATE = 3;                         // change the arm state to 3
                    } else if(gamepad2.b) {                     // Travel Pos to Lander
                        Extender.setPower(1);                   // extended the arm
                        motorOff();                             // motor off
                        sleep(5000);                // sleep for extending the arm
                        motorOn();                             // motor on
                        onPos = false;                         // tap the button and not hold it
                        pos = ((int) (TICK_PER_DEGREE * 20));  // dumping position
                        ARM_STATE = 5;                         // change the arm state to 5
                    }
                    break;

                case 3:                                         // En Route to Crater Pos

                    telemetry.addData("Pivot Position: ", Pivot.getCurrentPosition());
                    telemetry.addData("Math: ", (Pivot.getCurrentPosition() / TICK_PER_DEGREE));
                    telemetry.update();

                    if(Math.abs(Math.abs((Pivot.getCurrentPosition() / TICK_PER_DEGREE)) - 90) < 5) {
                                                                // check the arm position to be within 5 degrees of the 90 degree position
                        Extender.setPower(1);                   // extended the arm
                        onPos = false;                          // tap the button and not hold it
                        pos = ((int) (TICK_PER_DEGREE * 133));  // set pivot to mining state
                        ARM_STATE = 4;                          // change the arm state to 4
                    }
                    break;

                case 4:                                         // Crater Pos for mining
                    if(gamepad2.x) {
                        onPos = false;                          // tap the button and not hold it
                        pos = ((int) (TICK_PER_DEGREE * 105));  // horizontal position
                        ARM_STATE = 7;                          // change the arm state to 7
                    }
                    break;

                case 5:                                         // Lander Position transitioning to Travelling Position
                    if(gamepad2.y) {
                        onPos = false;                          // tap the button and not hoold it
                        pos = ((int) (TICK_PER_DEGREE * 77));   // travel position
                        ARM_STATE = 6;                          // change the arm state 6
                    }
                    break;

                case 6:                                         // Waiting for travel position before retracting the arm
                    telemetry.addData("Pivot Position: ", Pivot.getCurrentPosition());
                    telemetry.addData("Math: ", (Pivot.getCurrentPosition() / TICK_PER_DEGREE));
                    telemetry.update();

                    if(Math.abs(Math.abs((Pivot.getCurrentPosition() / TICK_PER_DEGREE)) - 77) < 5) {
                                                                // check the arm position to be within 5 degrees of the 90 degree position
                        Extender.setPower(0);                   // retract the arm to 0
                        ARM_STATE = 2;                          // change arm state to 2
                    }
                    break;

                case 7:                                         //Transition from crater to travel

                    if(Math.abs(Math.abs((Pivot.getCurrentPosition() / TICK_PER_DEGREE)) - 105) < 5) {
                                                                // Checking for arm position to be within 5 degrees of the 105 degree position
                        Extender.setPower(0);                   // retract arm
                        motorOff();                             // Motor off
                        sleep(2000);                // sleep for 2 seconds
                        motorOn();                              // Motor on
                        onPos = false;                          // tap button and not hold it
                        pos = ((int) (TICK_PER_DEGREE * 77));   // travel position
                        ARM_STATE = 2;                          // change arm state to state 2
                    }

            }
            rotateArm(pos);                                     // rotate arm to target position
        }
    }
                                                                // Code to rotate arm
        public void rotateArm(double targetPos)                 // rotates the arm to the target position given
        {




            if(!onPos)                                          // If the arm isn't at the position it should be, run the following -
            {
                Pivot.setTargetPosition((int)targetPos * -1);   // set the target position and make it positive
                Pivot.setPower(0.05);                           // setting power to 0.05
                if(Pivot.getCurrentPosition()==targetPos)       // checking if the current position is the same as the target position
                    onPos = true;                               // tap a button instead of hold
            }else 
            {
                Pivot.setPower(0);                              // set power 0 otherwise
            }

        }

                                                                // code to turn the power back on
        public void motorOn() {
            lFrontMotor.setPower(-gamepad1.left_stick_y * 0.8); // setting speed to 0.8
            lBackMotor.setPower(-gamepad1.left_stick_y * 0.8);  // setting speed to 0.8
            rFrontMotor.setPower(-gamepad1.right_stick_y * 0.8);// setting speed to 0.8
            rBackMotor.setPower(-gamepad1.right_stick_y * 0.8); // setting speed to 0.8
        }

        public void motorOff() {
            lFrontMotor.setPower(-gamepad1.left_stick_y * 0);   // setting speed to 0
            lBackMotor.setPower(-gamepad1.left_stick_y * 0);    // setting speed to 0
            rFrontMotor.setPower(-gamepad1.right_stick_y * 0);  // setting speed to 0
            rBackMotor.setPower(-gamepad1.right_stick_y * 0);   // setting speed to 0
        }
    }



