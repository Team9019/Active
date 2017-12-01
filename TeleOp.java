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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")

public class TeleOp extends LinearOpMode
{
    private HardRobot robot = new HardRobot();
    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor = null;
    private DcMotor armMotor = null;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo armServo;
    private Servo relicServo;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armServo= hardwareMap.get(Servo.class, "armServo");
        relicServo = hardwareMap.get(Servo.class, "relicServo");


        //  robot.Color.enableLed(true);
     //   robot.Color.enableLed(false);

        leftClaw.setPosition(0.9);
        rightClaw.setPosition(0.1);
        armServo.setPosition(0.02);
        relicServo.setPosition(0.2);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double liftPower;
            double slidePower;
            double armPower;

             leftPower  = -gamepad1.left_stick_y ;
             rightPower = -gamepad1.right_stick_y ;
             liftPower  = -gamepad2.left_stick_y ;
             slidePower = gamepad1.left_stick_x + gamepad1.right_stick_x ;
             armPower = gamepad1.right_trigger - gamepad1.left_trigger ;

            if (gamepad2.left_bumper)
            {
                robot.leftClaw.setPosition(0.1);
            }
            else
            {
                robot.leftClaw.setPosition(0.9);
            }
            if (gamepad2.right_bumper) {
                robot.rightClaw.setPosition(0.9);
            }
            else
            {
                robot.rightClaw.setPosition(0.1);
            }
            if (gamepad2.y)
            {
                armServo.setPosition(0.05);
            }
            if (gamepad2.x)
            {
                armServo.setPosition(0.02);
            }
            if (gamepad1.a)
            {
                relicServo.setPosition(0.5);
            }
            if (gamepad2.a)
            {
                relicServo.setPosition(0.2);
            }
            if (gamepad2.b)
            {
                armServo.setPosition(0.12);
            }


            // Send calculated power to wheels
            leftFront.setPower(Range.clip(leftPower + slidePower, -1.0, 1.0));
            rightFront.setPower(Range.clip(rightPower - slidePower, -1.0, 1.0));
            leftBack.setPower(Range.clip(leftPower - slidePower, -1.0, 1.0));
            rightBack.setPower(Range.clip(rightPower + slidePower, -1.0, 1.0));

            liftMotor.setPower(liftPower);
            armMotor.setPower(armPower);

            // Show the elapsed game time and wheel power.
           // telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower, slidePower, liftPower);
           // telemetry.update();
        }
    }
}
