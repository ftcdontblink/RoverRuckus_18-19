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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

class DRobot {
    //Declaring Motors
    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor rBackMotor;
    public DcMotor lBackMotor;
    public DcMotor Xmotor;
    public CRServo Extender;
    public Servo tmServo;
   // public DcMotor Grabber;
    public DcMotor Pivot;
    HardwareMap hwMap = null;
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
       // Grabber = hwMap.get(DcMotor.class, "Grabber");
        Pivot = hwMap.get(DcMotor.class, "Pivot");
        Extender = hwMap.get(CRServo.class, "Extender");

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

