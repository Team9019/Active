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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *   -
 *   -
 *   -
 *   -
 *   -
 *   -
 *   -
 */

@Autonomous(name="Red Right", group="Autonomous")

public class RedRight extends LinearOpMode
{
    /* Declare OpMode members. */
    private  HardRobot        robot   = new HardRobot();
    private  Commands         cmd     = new Commands();
    private ElapsedTime     runtime = new ElapsedTime();

//   // private static final double     DRIVE_SPEED             = 0.6;
//    private static final double     TURN_SPEED              = 0.5;
//    private static final double     LIFT_SPEED              = 0.7;
//    private static final double     JWL_DST                 = 3;
   private static boolean blueFound = false;
   private static boolean redFound = false;

    @Override
    public void runOpMode()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // 1) Drop Arm
        robot.colorServo.setPosition(0.73);


        // 2) Sense Color
        robot.Color.enableLed(true);
        runtime.reset();
        while ( opModeIsActive() && runtime.seconds() < 2)
        {
            telemetry.addData("SenseJewel", "Red:" + robot.Color.red());
            telemetry.addData("SenseJewel", "Blue:" + robot.Color.blue());
            telemetry.update();

            idle();
        }

        if (robot.Color.red() >= 1 && robot.Color.red() <= 100)
        {
            redFound = true;
        }
        if (robot.Color.blue() >= 1 && robot.Color.blue() <= 100)
        {
            blueFound = true;
        }
        robot.Color.enableLed(false);


        // 3) Lift Block
        robot.leftClaw.setPosition(0.1);
        robot.rightClaw.setPosition(0.9);
        sleep(200);

        cmd.encoderLift(cmd.LIFT_SPEED, 5, 2.0);


        // 4) Hit Jewel
        if (redFound)
        {
                cmd.encoderDrive(cmd.DRIVE_SPEED, -cmd.JWL_DST, -cmd.JWL_DST, 2.0);
        }
        if (blueFound)
        {
               cmd.encoderDrive(cmd.DRIVE_SPEED, cmd.JWL_DST, cmd.JWL_DST, 2.0);
        }


        // 5) Retract Arm
        robot.colorServo.setPosition(0.0);
        sleep(500);


        // 6) Forward/Right/Forward
        if (redFound) {
            cmd.encoderDrive(cmd.DRIVE_SPEED, cmd.JWL_DST + 24 , cmd.JWL_DST + 24, 4.0);
        }
        else if (blueFound) {
            cmd.encoderDrive(cmd.DRIVE_SPEED, -cmd.JWL_DST + 24, -cmd.JWL_DST + 24, 4.0);
        }

        sleep(200);
        cmd.encoderSlide(cmd.DRIVE_SPEED, 12, "L" , 2.0);
        sleep(200);
        cmd.encoderDrive(cmd.DRIVE_SPEED, 6, 6, 2.0);
        sleep(200);

        // 7) Place Block
        cmd.encoderLift(cmd.LIFT_SPEED, -3, 1.0);

        robot.leftClaw.setPosition(0.9);
        robot.rightClaw.setPosition(0.1);
        sleep(200);
        cmd.encoderDrive(cmd.DRIVE_SPEED, 2, 2, 1.0);

    }




}
