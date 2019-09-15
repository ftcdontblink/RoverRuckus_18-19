//IMPORTANT ----- THIS IS THE TELEOP CODE WITHOUT THE 

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Pushbot")
// @Disabled
public class TeleOp_New extends LinearOpMode {

    double k;
    //    // Declare OpMode members.
    DRobot robot = new DRobot();
    private ElapsedTime runtime = new ElapsedTime();
    boolean onPos = true;
    double pos = 0;


    static final double COUNTS_PER_MOTOR_REV = 24;
    static final double DRIVE_GEAR_REDUCTION = 60.0;
    static final double WHEEL_DIAMETER_INCHES = 1.25;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    static final double MOVE_SPEED = 0.1;

    static final double PIVOT_COUNTS_PER_MOTOR_REV = 28;
    static final double CHAIN_GEAR_REDUCTION = 40 / 15.0;
    static final double PIVOT_DRIVE_GEAR_REDUCTION = 40;
    static final double DEGREES_PER_REV = 360;
    static final double TICK_PER_DEGREE = (PIVOT_COUNTS_PER_MOTOR_REV * PIVOT_DRIVE_GEAR_REDUCTION * CHAIN_GEAR_REDUCTION) / (DEGREES_PER_REV);
    static final double TURN_SPEED = 0.2;
    static final double GRAB_SPEED = 0.3;
    static final double PIVOT_SPEED = 0.2;
    HardwareMap hwMap = null;

    // DECLARE MOTORS

    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor lBackMotor;
    public DcMotor rBackMotor;
    public DcMotor Xmotor;
    public DcMotor Pivot;
    public DcMotor Grabber;
    public CRServo Extender;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //initializing hardware map
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Initial setting", "? at %7d",
                robot.Xmotor.getCurrentPosition());
        telemetry.addData("Counts:", COUNTS_PER_INCH);
        telemetry.update();

        waitForStart();


        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            robot.lFrontMotor.setPower(-gamepad1.left_stick_y * 0.6);
            robot.lBackMotor.setPower(-gamepad1.left_stick_y * 0.6);
            robot.rFrontMotor.setPower(-gamepad1.right_stick_y * 0.6);
            robot.rBackMotor.setPower(-gamepad1.right_stick_y * 0.6);

            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            if (gamepad2.dpad_up) {

                    robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    telemetry.addData("dpad_up:", "on");
                    telemetry.update();
                    robot.Xmotor.setTargetPosition((int) (6.75 * COUNTS_PER_INCH));
                    robot.Xmotor.setPower(MOVE_SPEED);
                    robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }

                //When the left button on the Directional Pad is pressed, lift the latch to 5 inches (for aiming the final latch)

                if (gamepad2.dpad_left) {
                    robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("left:", "on");
                    telemetry.update();
                    robot.Xmotor.setTargetPosition((int) (5 * COUNTS_PER_INCH));
                    robot.Xmotor.setPower(MOVE_SPEED);
                    robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }

                // When the down button on the Directional Pad is pressed, bring the robot off the ground

                if (gamepad2.dpad_down) {
                    robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // encoderMove(MOVE_SPEED, -7.25, 10.00);
                    telemetry.addData("down:", "on");
                    telemetry.update();
                    robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.Xmotor.setTargetPosition((int) (3 * COUNTS_PER_INCH));
                    robot.Xmotor.setPower(0.3);

                }

                // When the right Button on the Directional Pad is pressed, bring the latch to its original position

                if (gamepad2.dpad_right) {
                    robot.Xmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // encoderMove(MOVE_SPEED, -7.25, 10.00);
                    telemetry.addData("right:", "on");
                    telemetry.update();
                    robot.Xmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.Xmotor.setTargetPosition(0);
                    robot.Xmotor.setPower(MOVE_SPEED);
                }

                    //If the a button on Gamepad 1 is pressed, set the robot to Full Power
                    if (gamepad1.a) {
                        robot.lFrontMotor.setPower(-gamepad1.left_stick_y * 1);
                        robot.lBackMotor.setPower(-gamepad1.left_stick_y * 1);
                        robot.rFrontMotor.setPower(-gamepad1.right_stick_y * 1);
                        robot.rBackMotor.setPower(-gamepad1.right_stick_y * 1);

                        telemetry.addData("Power =", "FULL");
                        telemetry.update();
                    }

                    //If the x button on gamepad 1 is pressed, set the power to 1/3


                    if (gamepad1.x) {

                        robot.lFrontMotor.setPower(-gamepad1.left_stick_y * 1 / 3);
                        robot.lBackMotor.setPower(-gamepad1.left_stick_y * 1 / 3);
                        robot.rFrontMotor.setPower(-gamepad1.right_stick_y * 1 / 3);
                        robot.rBackMotor.setPower(-gamepad1.right_stick_y * 1 / 3);

                        telemetry.addData("Power =", "0.333333...");
                        telemetry.update();
                    }

                    if (gamepad1.y) {
                    k = 0.6;
                    robot.lFrontMotor.setPower(-gamepad1.left_stick_y * 0.6);
                    robot.lBackMotor.setPower(-gamepad1.left_stick_y * 0.6);
                    robot.rFrontMotor.setPower(-gamepad1.right_stick_y * 0.6);
                    robot.rBackMotor.setPower(-gamepad1.right_stick_y * 0.6);
                    telemetry.addData("Power =", "0.6");
                    telemetry.update();
                }

                }

                
                


        

    }


    //Code to rotate arm
    public void rotateArm(double targetPos) {


        //If the arm isn't at the position it should be, run the following -
        if (!onPos) {
            Pivot.setTargetPosition((int) targetPos * -1);
            Pivot.setPower(0.05);
            if (Pivot.getCurrentPosition() == targetPos)
                onPos = true;
        } else {
            Pivot.setPower(0);
        }

    }
}