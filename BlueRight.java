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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 *
 *
 *
 *
 *
 *
 *
 *
 */

@Autonomous(name="Blue Right", group="Autonomous")

public class BlueRight extends LinearOpMode {

    /* Declare OpMode members. */
    private  HardRobot        robot   = new HardRobot();
    private  Commands         cmd     = new Commands(this, robot);
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 1140 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    private int colNo = 0;
    private int imageAdj = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
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

        senseImage(2);
        if (colNo == 1)
        {
            imageAdj = 6;
        }
        else if (colNo == 3)
        {
            imageAdj = (-6);
        }

        // 1) Drop Arm
        cmd.dropArm();

        // 2) Sense Color
        cmd.senseColor();

        // 3) Lift Block
        cmd.liftBlock();

        // 4) Hit Jewel
        if (cmd.redFound)
        {
                cmd.encoderDrive(cmd.DRIVE_SPEED, cmd.JWL_DST, cmd.JWL_DST, 2.0);
        }
        if (cmd.blueFound)
        {
               cmd.encoderDrive(cmd.DRIVE_SPEED, (-cmd.JWL_DST), (-cmd.JWL_DST), 2.0);
        }

        // 5) Retract Arm
        cmd.retractArm();

        // 6) Forward/Right/Forward
        if (cmd.redFound)
        {
            cmd.encoderDrive(cmd.DRIVE_SPEED, (cmd.JWL_DST + 24 + 4)*(-1) + (imageAdj), (cmd.JWL_DST + 24 + 4)*(-1) + (imageAdj), 4.0);
        }
        else if (cmd.blueFound)
        {
            cmd.encoderDrive(cmd.DRIVE_SPEED, (-cmd.JWL_DST + 24 + 4)*(-1) + (imageAdj), (-cmd.JWL_DST + 24 + 4)*(-1) + (imageAdj), 4.0);
        }

        cmd.encoderSlide(cmd.DRIVE_SPEED, 2, "L" , 2.0);
        cmd.encoderDrive(cmd.TURN_SPEED, 13, -13, 2.0);
        cmd.encoderDrive(cmd.DRIVE_SPEED, 6, 6, 2.0);

        // 7) Place Block
        cmd.placeBlock();
    }

    public int senseImage (double timeoutS)
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcI+TRj/////AAAAGUv8hVend0uFnM4Ru7qX3jVlRJ/McRWRQRwN8wHj00l9FqHhP+5CEKYpYNXs07Qng6Sw1ODIrS61iZiHxIye+6WAFbNYPmwo+1Lz4Dv8xyjxRofipuqYGRiPmkpMzffvDuui09EovmX26ifs74KVG5Zn7Xb6BaTS0wUadKFWlSFv73dQrDApmZGpd21bPe9Qv0Nrxhy9TN6Ztg3GQ0uoi1GRRpbTOSQ/Q9tBQJKuw17nfHZAkg+fJ3Jm33HV+DZUUNUpF6eiOFx2RL+xKOUlSLvg9c+VEZcHeY12PPl9docNYafMUJdZG2aDCASJWM6qbyjVN4OgIgOEyufTBOu5KBmejLMm/q+mE7m+2H1EVbOw";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //*********************************************************************************



        VuforiaLocalizer vuforia = null;
        VuforiaTrackables targets;
        VuforiaTrackable target;
        RelicRecoveryVuMark image;

        //Add reference to "robot." to parameters
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");
        target = targets.get(0);

        telemetry.addData("Action", "Sensing Crypto Key...");
        telemetry.update();

        targets.activate();

        runtime.reset();
        //Add check for colNo when moving to Commands.  This will prevent waiting the full timeout
        while (runtime.seconds()<timeoutS) // && colNo == 0)
        {
            image = RelicRecoveryVuMark.from(target);

            if (image == RelicRecoveryVuMark.LEFT)
            {
                colNo = 1;
            }
            else if (image == RelicRecoveryVuMark.CENTER)
            {
                colNo = 2;
            }
            else if (image == RelicRecoveryVuMark.RIGHT)
            {
                colNo = 3;
            }

            if (colNo != 0)
            {
                telemetry.addData("Detected", "Key found for column " + colNo);
            }
            else
            {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
        targets.deactivate();

        telemetry.addData("Action", "Sensing Crypto Key Complete!");
        telemetry.update();

        return colNo;
    }
}
