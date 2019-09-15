/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Speed_toggle", group="Linear Opmode")
// @Disabled
public class Speed_toggle extends LinearOpMode {

    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor rBackMotor;
    public DcMotor lBackMotor;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();


    public void init (HardwareMap aHwMap){
        hwMap = aHwMap;
        //inititalize motors
        lFrontMotor = hwMap.get(DcMotor.class, "left_Front_Motor");
        rFrontMotor = hwMap.get(DcMotor.class, "right_Front_Motor");
        lBackMotor = hwMap.get(DcMotor.class, "left_Back_Motor");
        rBackMotor = hwMap.get(DcMotor.class, "right_Back_Motor");
    }
    // Set the robot
    DRobot robot = new DRobot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize the robot
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a) {
                setSpeed(0.6);
                telemetry.addData("Speed mode: ", "move");
                telemetry.update();
            } else if(gamepad1.b) {
                setSpeed(0.3);
                telemetry.addData("Speed mode: ", "slow");
                telemetry.update();
            } else if(gamepad1.x) {
                setSpeed(0.1);
                telemetry.addData("Speed mode: ", "turtle");
                telemetry.update();
            }
        }

    }

    public void setSpeed(double speed) {
        lFrontMotor.setPower(-gamepad1.left_stick_y * speed);
        lBackMotor.setPower(-gamepad1.left_stick_y * speed);
        rFrontMotor.setPower(-gamepad1.right_stick_y * speed);
        rBackMotor.setPower(-gamepad1.right_stick_y * speed);
    }
}
