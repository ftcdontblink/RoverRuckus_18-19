package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class DRobot {
    //Declaring Motors
    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor rBackMotor;
    public DcMotor lBackMotor;
    public DcMotor Xmotor;
   //public Servo Extender;
    public Servo tmServo;
   // public DcMotor Grabber;
    //public DcMotor combo;
    HardwareMap hwMap = null;
    public Servo armMotor = null;
    public ModernRoboticsI2cColorSensor frontColorSensor = null;
    public ModernRoboticsI2cGyro gyroSensor = null;

    public static double latch_free, latch_park, latch_down;

    //Default Constructor
    public DRobot(){}

    public void init (HardwareMap aHwMap){
        hwMap = aHwMap;
        //inititalize motors
        lFrontMotor = hwMap.get(DcMotor.class, "left_Front_Motor");
        rFrontMotor = hwMap.get(DcMotor.class, "right_Front_Motor");
        lBackMotor = hwMap.get(DcMotor.class, "left_Back_Motor");
        rBackMotor = hwMap.get(DcMotor.class, "right_Back_Motor");
        Xmotor = hwMap.get(DcMotor.class, "Xmotor");
        tmServo = hwMap.get(Servo.class, "TMServo");
       // grabber = hwMap.get(DcMotor.class, "Grabber");
       // Extender = hwMap.get(Servo.class, "Extender");

        //Set left motors to reverse
        rBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rFrontMotor.setDirection(DcMotor.Direction.REVERSE); 

        //rFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset motor encoders.
//        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //set motor power to zero.
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
    }

}

