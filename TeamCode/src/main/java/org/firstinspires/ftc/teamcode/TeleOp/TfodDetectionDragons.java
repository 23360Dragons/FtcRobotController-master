    package org.firstinspires.ftc.teamcode.TeleOp;
    
    import android.util.Size;
    
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.robotcore.hardware.Servo;
    //import com.qualcomm.robotcore.hardware.TouchSensor;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;
    
    import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
    import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
    import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
    import org.firstinspires.ftc.vision.tfod.TfodProcessor;
    
    import java.util.List;
    import java.util.concurrent.TimeUnit;
    @Autonomous
    public class TfodDetectionDragons extends LinearOpMode {
    
        // Adjust these numbers to suit your robot.
        final double DESIRED_DISTANCE = 10; //  this is how close the camera should get to the target (inches)
    
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response
    
        final double SPEED_GAIN  =  0.03  ;   // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   // Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =  0.01  ;   // Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    
        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
        private DcMotor _intake = null;
        private DcMotor _linear1 = null;
        private DcMotor _linear2 = null;
        private Servo _claw1 = null;
        private Servo _claw2 = null;
        IMU _orient;
        final static double Claw_Home = 0.0;
        private double cpos1 = 0.0;
        private double cpos2 = 0.0;
        private Servo _arm1 = null;
        private Servo _arm2 = null;
        private double pos1 = 0.0;
        private double pos2 = 0.0;
        private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
        private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
        private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
        private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    
        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
        private int DESIRED_TAG_ID = 0;     //  Choose the tag you want to approach or set to -1 for ANY tag.
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
        private int     myExposure  ;
        private int     myGain      ;
    
        /**
         * The variable to store our instance of the TensorFlow Object Detection processor.
         */
        private TfodProcessor tfod;
    
        //TouchSensor touchSensor;  // Touch sensor Object
    
        // servo variables
        static final double INCREMENT   = 0.08;     // .01 amount to slew servo each CYCLE_MS cycle
        static final int    CYCLE_MS    =   50;     // period of each cycle
        static final double MAX_POS     =  1.0;     // Maximum rotational position
        static final double MIN_POS     =  0.25;     // 0.0 Minimum rotational position
    
        @Override
        public void runOpMode() throws InterruptedException {
            YawPitchRollAngles robotOrientation;
            robotOrientation = _orient.getRobotYawPitchRollAngles();
            double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            boolean pixelFound      = false;    // set to true when pixel found on spike mark
            String pixelLocation   = "";        // set to left, centre, or right
            double  drive           = 0;        // Desired forward power/speed (-1 to +1)
            double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
            double  turn            = 0;        // Desired turning power/speed (-1 to +1)
            int     currentStep            = 1;
            ElapsedTime runtime = new ElapsedTime();
    
            initTfod();
    
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must match the names assigned during the robot configuration.
            // step (using the FTC Robot Controller app on the phone).
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFront");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFront");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBack");
            rightBackDrive = hardwareMap.get(DcMotor.class, "RightBack");

            _orient = hardwareMap.get(IMU.class, "BHI260AP");
    
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

            // Note: the line above this is the longest in the code lol
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            //get the intake motor
            _intake = hardwareMap.get(DcMotor.class, "Intake");
            // get a reference to our touchSensor object.
            //touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
            //map the linear slide with the DcMotor
            _linear1 = hardwareMap.get(DcMotor.class, "Linear1");
            _linear2 = hardwareMap.get(DcMotor.class, "Linear2");

            _linear2.setDirection(DcMotor.Direction.REVERSE);

            _linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            _linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Resets encoders to zero
            _linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Turns encoders back on
            _linear1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            _linear2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            _claw1 = hardwareMap.get(Servo.class, "Claw1");
            _claw2 = hardwareMap.get(Servo.class, "Claw2");

            _claw1.setPosition(Claw_Home);
            _claw2.setPosition(Claw_Home);
            _arm1 = hardwareMap.get(Servo.class, "Arm1");
            _arm2 = hardwareMap.get(Servo.class, "Arm2");
            _arm2.setDirection(Servo.Direction.REVERSE);
            pos1 = 0.5;
            pos2 = 0.5;
            _arm2.setPosition(pos2);
            _arm1.setPosition(pos1);
            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();
    
            runtime.reset();  // start timer for step 1
            while (opModeIsActive()) {
                targetFound = false;
                desiredTag  = null;
    
                // STEP 1 move forward
                if (currentStep==1) {
                    if (runtime.milliseconds() < 250) {
                        moveRobot(-0.75, 0, 0);// originally x was 10, then was 3, then 0.75
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 2;
                        runtime.reset();  // start timer for step 2
                    }
                }


    
                // STEP 2 slight rotation counter-clockwise - look at left spike mark
                // yaw was positive 2, but it was looking in the wrong direction
                // made yaw negative 2
                // +made yaw negative 1
                if (currentStep==2) {
                    if (runtime.milliseconds() < 175) {
                        moveRobot(0, 0, -1);//yaw5
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 3;
                        runtime.reset();  // start timer for step 3
                    }
                }

    
                // STEP 3 use Tensorflow to check for pixel on left spike mark, allow 5 seconds to elapse
                if (currentStep==3) {
                    if (runtime.milliseconds() < 3000) {
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        // Step through the list of recognitions and look for pixel
                        for (Recognition recognition : currentRecognitions) {
                            if (recognition.getLabel() == "Pixel") {
                                currentStep = 4;
                                pixelLocation = "left";
                                pixelFound = true;
                                telemetry.addData("found on spike mark ", pixelLocation);
                                runtime.reset();  // start timer for step 4
                            } else {
                                sleep(50);
                            }
                        }   // end for() loop
                    } else {
                        // pixel not found, try center spike mark
                        currentStep = 6;
                        runtime.reset();  // start timer for step 6
                    }
                }


                // STEP 4 move forward towards left spike mark
                if (currentStep==4) {
                    //need to experiment with value of X
                    if (runtime.milliseconds() < 460) {
                        moveRobot(-0.4, 0, 0);
                        _intake.setPower(-0.3);
                        Thread.sleep(30);
                        _intake.setPower(0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 31;
                        runtime.reset();  // start timer for step 5
                    }
                }
    
                // STEP 5 dropping off purple pixel
                //need to experiment to see if we need a positive or negative value
                //need to experiment with the amount time this runs, current 460ms
                if (currentStep==5) {
                    if (runtime.milliseconds() < 460) {
                        moveRobot(-0.5, 0, 0);
                    } else {
                        _intake.setPower(0);
                        currentStep = 7;  // point at backdrop
                        runtime.reset();  // start timer for step 7
                    }
                }



                // STEP 6 slight rotation clockwise to check centre spike mark
                if (currentStep==6) {
                    if (runtime.milliseconds() < 175) {
                        moveRobot(0, 0, 2.5);
                        //Need to change to turn more FINDCENTRE
                    }
                    else if (runtime.milliseconds() > 175 && runtime.milliseconds() < 300) {
                        //move forward
                        moveRobot(-1, 0, 0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 10;
                        runtime.reset();  // start timer for step 10
                    }
                }


                // STEP 7 bigger rotation clockwise from left spike mark to the backdrop
                //need to experiment with the value of yaw and the time of 750ms
                //the goal is to have the robot pointed at the backdrop at the correct location

                if (currentStep==7) {

                    if (runtime.milliseconds() < 750) {
                        moveRobot(0, 0, -5);//yaw-5
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 20;    // head to backdrop
                        runtime.reset();  // start timer for step 10
                    }
                }
    
                // STEP 10 - use Tensorflow to check for pixel on centre spike mark, allow 5 seconds to elapse
                if (currentStep==10) {
                    if (runtime.milliseconds() < 3000) {
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        // Step through the list of recognitions and look for pixel
                        for (Recognition recognition : currentRecognitions) {
                            if (recognition.getLabel() == "Pixel") {
                                currentStep = 11; // drop off at centre
                                pixelLocation = "center";
                                pixelFound = true;
                                telemetry.addData("found on spike mark ", pixelLocation);
                                runtime.reset();  // start timer for step 11
                            } else {
                                sleep(50);
                            }
                        }   // end for() loop
                    } else {
                        // pixel not found, assume right spike mark
                        pixelLocation = "right";
                        telemetry.addData("found on spike mark ", pixelLocation);
                        currentStep = 15; // drop off on right mark
                        runtime.reset();  // start timer
                    }
                }


                // STEP 11 - move forward towards centre spike mark
                if (currentStep==11) {
                    if (runtime.milliseconds() < 500) {
                        //was x 5, y 0 , yaw 0
                        //made it x 0 y 0 yaw 0 to stop the movement
                        moveRobot(-0.4, 0, 0);
                        //set current step to 31 to end program
                        _intake.setPower(-0.3);
                        Thread.sleep(30);
                        _intake.setPower(0);
                        currentStep = 12;
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 12;
                        runtime.reset();  // start timer for step 12
                    }
                }


                // STEP 12 backoff from centre spike mark, dropping off purple pixel
                if (currentStep==12) {
                    if (runtime.milliseconds() < 500) {
                        moveRobot(-0.5, 0, 0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 31;    // turn toward backdrop
                        runtime.reset();  // start timer for step 13
                    }
                }


                // STEP 13 bigger rotation clockwise from center mark to backdrop
                if (currentStep==13) {
                    if (runtime.milliseconds() < 500) {
                        //was x 0 y 0 yaw -5
                        //made it x 0 y 0 yaw 0 to stop movement
                        moveRobot(0, 0, -0);
                        //set current step to 31 to stop program
                        currentStep = 31;
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 20;    // head to backdrop
                    }
                }


                // STEP 15 - turn towards right spike mark
                if (currentStep==15) {
                    if (runtime.milliseconds() < 200) {
                        //was x 0 y 0 yaw -5
                        //made x 0 y 0 yaw 0 to stop movement
                        moveRobot(0, 0, -5);
                        //set current step to 31 to end program
                        currentStep = 16;
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 16;  // move towards mark
                        runtime.reset();  // start timer
                    }
                }


                // STEP 16  move forward towards right spike mark
                if (currentStep==16) {
                    if (runtime.milliseconds() < 480) {
                        //was x 5, y 0 , yaw 0
                        //made x 0 y 0 yaw 0 to stop movement
                        moveRobot(-0.5, 0, 0);
                        _intake.setPower(-0.3);
                        Thread.sleep(30);
                        _intake.setPower(0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 17;
                        runtime.reset();  // start timer for step 17
                    }
                }
    
                // STEP 17  move backward from right spike mark
                if (currentStep==17) {
                    if (runtime.milliseconds() < 480) {
                        moveRobot(-5, 0, 0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 31;
                        runtime.reset();  // start timer for step 18
                    }
                }


                // STEP 18 - point webcam 2 at background
                if (currentStep==18) {
                    if (runtime.milliseconds() < 250) {
                        moveRobot(0, 0, -5);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 20;  // go to backdrop
                        //runtime.reset();  // start timer
                    }
                }


                // STEP 20 - use April Tags to move toward backdrop
                //  start by closing TFOD and starting April Tag
                if (currentStep==20) {
                    visionPortal.close();
                    sleep(50);
                    // Initialize the Apriltag Detection process
                    initAprilTag();
                    sleep(50);
                    setManualExposure(20, 250);  // Use low exposure time high gain to reduce motion blur

                    if (pixelLocation == "left") {
                        DESIRED_TAG_ID = 4;
                    } else if (pixelLocation == "centre"){
                        DESIRED_TAG_ID = 5;
                    } else DESIRED_TAG_ID = 6;
    
                    currentStep = 22;
                    runtime.reset();  // timer for step 21
                }


                // STEP 22 - move to backdrop using April Tag
                if (currentStep==22) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)){
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        }
                    }
    
                    if (targetFound) {
                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double  headingError    = desiredTag.ftcPose.bearing;
                        double  yawError        = desiredTag.ftcPose.yaw;
                        if ((rangeError < 7) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)) {
                            // if we're close enough, stop using April Tag logic
                            drive = 0;
                            turn = 0;
                            strafe = 0;
                            currentStep = 23;  // drive to backdrop
                            runtime.reset();
                        } else {
                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                        }
                        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    
                        // Apply desired axes motions to the drivetrain.
                        moveRobot(drive, strafe, turn);
                        sleep(10);
                    } else {
                        moveRobot(0, 0, 0);
                        sleep(10);
                    }
                }
    
                // STEP 23
                //note, this was originally using a touch sensor to detect when the robot reached the backdrop
                //we do not currently have a touch sensor.
                //we will simply move the robot and have to experiment with how long to move it
                //if you need to see the touch sensor code, reference TfodDetectionReference.java
                if (currentStep==23)
                {
                    //move the robot to the backdrop
                    //need to experiment with the X value and the time value of 250ms to see if we are moving the right amount
                    if (runtime.milliseconds() < 250) {
                        moveRobot(-3, 0, 0);
                    } else {
                        moveRobot(0, 0, 0);
                        currentStep = 24;
                        runtime.reset();  // start timer for step 24
                    }
                }
    
                // STEP 24 deploy arm with yellow pixel to backdrop
                // extend linear slide
                if (currentStep==24) {

                    //need to experiment with time of 250ms and power = 1
                    if(runtime.milliseconds()<250) {
                            _linear1.setPower(0.5);
                            _linear2.setPower(0.5);
                    } else {
                        _linear1.setPower(0);
                        _linear2.setPower(0);
                    }

                }

                // STEP 25 swing arm
                //need to experiment with what position to set this to to deploy
                //need to experiment if this moves the arm correctly
                if (currentStep==25) {
                    pos1 = 0.45;
                    pos2 = 0.45;
                    _arm1.setPosition(pos1);
                    _arm2.setPosition(pos2);

                        currentStep = 26;
                        sleep(CYCLE_MS);  // let servo have time to move
                }


                //open the claw and deploy the yellow pixel
                if(currentStep==26) {
                    cpos1 = 0.35;
                    cpos2 = 0.35;
                    _claw1.setPosition(cpos1);
                    _claw2.setPosition(cpos2);
                        //claw should now be opened
                        currentStep = 27;

                    sleep(CYCLE_MS);  // let servo have time to move
                }


                //close the claw and deploy the yellow pixel
                if(currentStep==27) {
                    cpos1 -= 0.02;
                    cpos2 -= 0.02;

                    if(cpos1 <= 0.0) {
                        //claw should now be closed
                        currentStep = 28;
                    } else {
                        _claw1.setPosition(cpos1);
                        _claw2.setPosition(cpos2);
                    }

                    sleep(CYCLE_MS);  // let servo have time to move
                }


                //retract the arm
                if(currentStep==28)
                {
                    pos1 = 0.65;
                    pos2 = 0.65;
                    _arm1.setPosition(pos1);
                    _arm2.setPosition(pos2);
                        //arm should now be deployed
                        currentStep = 30;
                    sleep(CYCLE_MS);  // let servo have time to move
                }



                // STEP 30 - move to the inside and park
                if (currentStep==30) {
                    if (runtime.milliseconds() < 500) {
                        moveRobot(0, -5, 0);
                        sleep(10);
                    }
                    else { // end of autonomous, robot will do nothing else
                        moveRobot(0, 0, 0);
                        sleep(10);
                    }
                }
    
                telemetry.addData("current step", currentStep);
                telemetry.addData("pixel found", pixelFound);
                telemetry.addData("pixel location", "%s", pixelLocation);
                telemetry.addData("tag target", DESIRED_TAG_ID);
                telemetry.addData("tag found", targetFound);
                telemetry.update();
            }
            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();
        }
        /**
         * Initialize the TensorFlow Object Detection processor.
         */
        private void initTfod() {
            // Create the TensorFlow processor by using a builder.
    
            tfod = TfodProcessor.easyCreateWithDefaults();
    
            //tfod = new TfodProcessor.Builder()
                    // Use setModelAssetName() if the TF Model is built in as an asset
                    // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                    //.setModelAssetName(TFOD_MODEL_ASSET)
                    //.setModelFileName(TFOD_MODEL_FILE)
                    //.setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)
                    //.build();
            tfod.setZoom(1.5);
    
            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();
    
            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }
    
            // Choose a camera resolution. Not all cameras support all resolutions.
            builder.setCameraResolution(new Size(1280, 720));
    
            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            //builder.enableCameraMonitoring(true);
    
            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
    
            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);
    
            // Set and enable the processor.
            builder.addProcessor(tfod);
    
            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
    
            // Set confidence threshold for TFOD recognitions, at any time.
            tfod.setMinResultConfidence(0.75f);
    
            // Disable or re-enable the TFOD processor at any time.
            //visionPortal.setProcessorEnabled(tfod, true);
        }   // end method initTfod()
    
        /**
         * Move robot according to desired axes motions
         * Positive X is forward
         * Positive Y is strafe left
         * Positive Yaw is counter-clockwise
         */
        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            //changed power calculations by adding *0.7 to run at 70% of what it ran at beforehand
            double leftFrontPower    =  (x -y -yaw) *0.7;
            double rightFrontPower   =  (x +y +yaw) *0.7;
            double leftBackPower     =  (x +y -yaw) *0.7;
            double rightBackPower    =  (x -y +yaw) *0.7;
    
            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
    
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
    
            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    //.setLensIntrinsics(3164.64, 3164.64, 1935.8, 1092.57) // hires webcam
                    .setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157) // logitech 920
                    .build();
    
            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                        .setCameraResolution(new Size(1280, 720))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }
    
        /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls
    
            if (visionPortal == null) {
                return;
            }
    
            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }
    
            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
    }
