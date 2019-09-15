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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Auto_Depot", group="Pushbot")

/**
 *Auton_Depot is an autonomous code that uses
 */

public class Auto_Depot extends LinearOpMode {

    //Declare the robot
    DRobot robot = new DRobot();

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 5.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH =(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/ (Math.PI*WHEEL_DIAMETER_INCHES);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;
    static final double     LATCH_SPEED             = 0.15;

    static final double     COUNTS_PER_MOTOR_REV_latch    = 24;
    static final double     DRIVE_GEAR_REDUCTION_latch    = 60.0;
    static final double     WHEEL_DIAMETER_INCHES_latch   = 1.25;
    static final double     COUNTS_PER_INCH_latch =(COUNTS_PER_MOTOR_REV_latch * DRIVE_GEAR_REDUCTION_latch)/ (Math.PI*WHEEL_DIAMETER_INCHES_latch);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //vuforia key
    private static final String VUFORIA_KEY = "AVkZDiv/////AAABmeaYHMdP4k7BuPrxZYfopLZlyb5Mi11PmL2rjUEyj4AAAKjmOPE0oTk+TlU8KALCll5GrCGyQbwaH/Zge1WMHHouz7OtMBdy3c5ZEr0HKhTFYoCYxEJqhlJUa3CAt1koS+lQL6gEbWVhVkWd3WVrpm+zq/yD1lfXIblX0aiBgN3vS0oAGQ6K/VcYHpQp2IeadE83io4wiq8ogASt/8Q5oP8AfHNPeU0acDtQuCxBeo0y99c014GJHeOTnb1NxGjsjDZIOXWtJOMIY2WUtllA2jXZZ84IHUpseNGiWLfA8t1pEgDWfh/FxgsE2hAf7NBHVKbGGZ6OW9LdXduHfGhgBU0F8uy1ePEi3vZFLwes1uR8";

    //Declare Vuforia localization engine
    private VuforiaLocalizer vuforia;

    //Declare tensorflow object detector
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        robot.init(hardwareMap);

        telemetry.addData("Latch Motor", "Resetting Encoders");    //
        telemetry.update();

        

        telemetry.addData("Latch Power", "-0.2");    //
        telemetry.update();


        //set the wheel motors ready
        robot.lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Initial setting","Starting at %7d:%7d:%7d:%7d",
                robot.lFrontMotor.getCurrentPosition(),
                robot.lBackMotor.getCurrentPosition(),
                robot.rBackMotor.getCurrentPosition(),
                robot.rFrontMotor.getCurrentPosition());
        telemetry.addData("Counts:", COUNTS_PER_INCH  );
        telemetry.update();

        //initializing tensorflow
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        robot.tmServo.setPosition(0);
        //Robot in park position
        //robot.Xmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Xmotor.setTargetPosition(0);
        robot.Xmotor.setPower(-0.2);


        waitForStart();
        //Unlatching the robot from the lander
        extendLatch(LATCH_SPEED, 9, 10.0);
        robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("UnLatching", "Complete");
        telemetry.update();
//        robot.Xmotor.setTargetPosition(0);
//        robot.Xmotor.setPower(-0.2);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        //check if all three minerals are visible
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            //identify the position of the gold
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            //move the robot in the direction of the gold mineral
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    Depot_Left();
                                }
                                else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    Depot_Right();

                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    Depot_Middle();
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        //close tensor flow
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    //The tasks the robot complete if it finds the gold mineral in the left position
    public void Depot_Left()
    {
        encoderDrive(DRIVE_SPEED, 3, 3, 4.0);
        encoderTurn(TURN_SPEED, 50, 4.0);
        encoderDrive(DRIVE_SPEED, 19, 19, 4.0);
        encoderTurn(TURN_SPEED, -65, 4.0);
        encoderDrive(DRIVE_SPEED, 18, 18, 4.0);
        placeTM();
        encoderDrive(DRIVE_SPEED, -2, -2, 4.0);
//        encoderDrive(DRIVE_SPEED, -27, -27, 4.0);
//        encoderDrive(DRIVE_SPEED, 8.5, -8.5, 4.0);
//        encoderDrive(DRIVE_SPEED, 32, 32, 4.0);
//        encoderTurn(TURN_SPEED, -40, 4.0);
//        encoderDrive(DRIVE_SPEED, 12, 12, 4.0);
    }

   //The tasks the robot complete if it finds the gold mineral in the middle position
    public void Depot_Middle()
    {
        encoderDrive(DRIVE_SPEED,  30,  30, 5.0); //move forward towards depot
        sleep(1000);
        placeTM();
        sleep(1000);
//        encoderDrive(DRIVE_SPEED,  -16,  -16, 5.0);//move backwards towards lander
//        encoderTurn(TURN_SPEED, 103, 2.0); //Turn 90 degrees to the left
//        encoderDrive(DRIVE_SPEED,  -35,  -35, 5.0); //Move backwards towards Crater
//        encoderTurn(TURN_SPEED, -45, 2.0); //Turn 90 degrees to the left
//        encoderDrive(DRIVE_SPEED,  -12,  -12, 5.0); //Move backwards towards Crater


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    //The tasks the robot complete if it finds the gold mineral in the right position
    public void Depot_Right(){
        encoderDrive(DRIVE_SPEED, 3, 3, 4.0);
        encoderTurn(TURN_SPEED, -45, 2.0);
        encoderDrive(DRIVE_SPEED, 10, 10, 4.0);
        encoderTurn(TURN_SPEED, 20, 2.0);
        encoderDrive(0.2, 8, 8, 4.0);
        encoderDrive(DRIVE_SPEED, -7, -7, 4.0);
        encoderTurn(TURN_SPEED, -30, 2.0);
        encoderDrive(DRIVE_SPEED, 10, 10, 4.0);
        encoderTurn(TURN_SPEED, 60, 2.0);
        encoderDrive(DRIVE_SPEED, 15, 15, 4.0);
        encoderTurn(TURN_SPEED, 40, 1.0);
        placeTM();
        encoderDrive(DRIVE_SPEED, -60,-60,5.0);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    //Move the robot to the desired target position using encoder
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.lFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.lBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rBackMotor.setTargetPosition(newRightBackTarget);
            robot.rFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lFrontMotor.setPower(Math.abs(speed));
            robot.lBackMotor.setPower(Math.abs(speed));
            robot.rFrontMotor.setPower(Math.abs(speed));
            robot.rBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lFrontMotor.isBusy() && robot.lBackMotor.isBusy() || robot.rFrontMotor.isBusy() && robot.rBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
                        robot.lFrontMotor.getCurrentPosition(),
                        robot.lBackMotor.getCurrentPosition(),
                        robot.rBackMotor.getCurrentPosition(),
                        robot.rFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.lFrontMotor.setPower(0);
            robot.lBackMotor.setPower(0);
            robot.rFrontMotor.setPower(0);
            robot.rBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    // Turn the robot to the desired target position using encoders
    public void encoderTurn(double speed, int turn,
                            double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.lFrontMotor.getCurrentPosition() - turn;
            newRightFrontTarget = robot.rFrontMotor.getCurrentPosition() + turn;
            newLeftBackTarget = robot.lBackMotor.getCurrentPosition() - turn;
            newRightBackTarget = robot.rBackMotor.getCurrentPosition() + turn;
            robot.lFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.lBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rBackMotor.setTargetPosition(newRightFrontTarget);
            robot.rFrontMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lFrontMotor.setPower(Math.abs(speed));
            robot.lBackMotor.setPower(Math.abs(speed));
            robot.rFrontMotor.setPower(Math.abs(speed));
            robot.rBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when all motors hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lFrontMotor.isBusy() && robot.lBackMotor.isBusy() || robot.rFrontMotor.isBusy() && robot.rBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.lFrontMotor.getCurrentPosition(),
                        robot.lBackMotor.getCurrentPosition(),
                        robot.rBackMotor.getCurrentPosition(),
                        robot.rFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.lFrontMotor.setPower(0);
            robot.lBackMotor.setPower(0);
            robot.rFrontMotor.setPower(0);
            robot.rBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    // Extend the latching mechanism to unlatch the robot from the lander
    public void extendLatch(double speed,
                            double inches,
                            double timeoutS) {

        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = robot.Xmotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_latch);
            robot.latch_free = target;
            robot.Xmotor.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Xmotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.Xmotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Running at %7d" ,
                        robot.Xmotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    //Placing the team marker into the depot
    public void placeTM (){
        double servoPosition = 0.75;
        robot.tmServo.setPosition(servoPosition);
    }
}

