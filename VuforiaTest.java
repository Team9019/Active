package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

//Add to Commands
//*********************************************************************************
    import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//*********************************************************************************

@Autonomous(name="Vuforia Test", group ="Test")
//@Disabled
public class VuforiaTest extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //Add to each autonomous
        int colNo=0;

    @Override public void runOpMode()
    {
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        //Call vuforia sensing from each autonomous using reference to "cmds.senseImage"
            colNo = senseImage(20);

            //Take action in each autonomous, based on ColNo returned
            telemetry.addData(">", "Make movement adjustment for column:  " + colNo);
            telemetry.update();
            sleep(2000);
    }

    //*********************************************************************************
    //Add the entire senseImage method to the Commands class
    public int senseImage (double timeoutS)
    {
        //*********************************************************************************
        //Move to HardRobot

            //In competition, no need to show the image on the RC.  Turn monitor off to save power
                //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            //While testing, display the image on the RC
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AcI+TRj/////AAAAGUv8hVend0uFnM4Ru7qX3jVlRJ/McRWRQRwN8wHj00l9FqHhP+5CEKYpYNXs07Qng6Sw1ODIrS61iZiHxIye+6WAFbNYPmwo+1Lz4Dv8xyjxRofipuqYGRiPmkpMzffvDuui09EovmX26ifs74KVG5Zn7Xb6BaTS0wUadKFWlSFv73dQrDApmZGpd21bPe9Qv0Nrxhy9TN6Ztg3GQ0uoi1GRRpbTOSQ/Q9tBQJKuw17nfHZAkg+fJ3Jm33HV+DZUUNUpF6eiOFx2RL+xKOUlSLvg9c+VEZcHeY12PPl9docNYafMUJdZG2aDCASJWM6qbyjVN4OgIgOEyufTBOu5KBmejLMm/q+mE7m+2H1EVbOw";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //*********************************************************************************

        int colNo=0;

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
    //*********************************************************************************
}
