package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.jar.Attributes;

//demo GITHUB 8*24*2024

//@Autonomous
//public class AutoCV extends LinearOpMode
//{
    //OpenCvWebcam webcam;
    //SamplePipeline pipeline;
    //@Override
    //public void runOpMode() throws InterruptedException
    //{
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //WebcamName webcamName = null;
        //webcamName = hardwareMap.get(WebcamName.class, "WebcamMain"); // put your camera's name here
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
    //}



        /*
        CenterStageCVdetection detector = new CenterStageCVdetection(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000);
         */
       /* camera.openCameraDeviceAsync(new OPenCVCamera.AsyncCameraOpenListener()
        {
            @override
            public void onOpened()
            {
                /* Tell Webcam to start streaming images
                 *//*
            }
        })

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();
        switch();
        switch (detector.getLocation())
        {
            case Left;
                //...
                break;
            case right;
                // ...
                break
            case middle;
                // ..
                break;
        }
        camera.stopStreaming();
    }

    */
