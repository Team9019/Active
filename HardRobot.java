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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class HardRobot
{
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor liftMotor = null;
    public DcMotor armMotor = null;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo lunderClaw;
    public Servo runderClaw;
    public Servo colorServo;
    public ModernRoboticsI2cColorSensor Color;
//    public int cameraMonitorViewId;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardRobot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;
 //        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");

        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        lunderClaw = hwMap.get(Servo.class, "lunderClaw");
        runderClaw = hwMap.get(Servo.class, "runderClaw");
        colorServo = hwMap.get(Servo.class, "colorServo");
        Color = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("Color");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        Color.enableLed(true);
        Color.enableLed(false);

        leftClaw.setPosition(0.9);
        rightClaw.setPosition(0.1);
        lunderClaw.setPosition(0.4);
        runderClaw.setPosition(0.3);
        colorServo.setPosition(0.1);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        liftMotor.setPower(0);

        //*********************************************************************************
        //Move to HardRobot

        //In competition, no need to show the image on the RC.  Turn monitor off to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //While testing, display the image on the RC
       // int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

//        parameters.vuforiaLicenseKey = "AcI+TRj/////AAAAGUv8hVend0uFnM4Ru7qX3jVlRJ/McRWRQRwN8wHj00l9FqHhP+5CEKYpYNXs07Qng6Sw1ODIrS61iZiHxIye+6WAFbNYPmwo+1Lz4Dv8xyjxRofipuqYGRiPmkpMzffvDuui09EovmX26ifs74KVG5Zn7Xb6BaTS0wUadKFWlSFv73dQrDApmZGpd21bPe9Qv0Nrxhy9TN6Ztg3GQ0uoi1GRRpbTOSQ/Q9tBQJKuw17nfHZAkg+fJ3Jm33HV+DZUUNUpF6eiOFx2RL+xKOUlSLvg9c+VEZcHeY12PPl9docNYafMUJdZG2aDCASJWM6qbyjVN4OgIgOEyufTBOu5KBmejLMm/q+mE7m+2H1EVbOw";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //*********************************************************************************

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        // Define and initialize ALL installed servos.
        //  leftClaw  = hwMap.get(Servo.class, "left_hand");
        // rightClaw = hwMap.get(Servo.class, "right_hand");
        // leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);
    }
 }

