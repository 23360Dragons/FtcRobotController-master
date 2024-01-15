package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class ProgramStorage  {

}


    // Label all programs to be stored to work on later please

    /*                        Claw program for now
      The claw will open and close when pressing x and y using servos idk
      if it'll work cause I think theres something else but I cant remember
      what

        Servo claw1;
    Servo claw2;
    final static double Claw_Home = 0.0;
    final static double Claw_Min = 0.0;
    final static double Claw_Max = 0.0;












    @Override
    public void init() {

        claw1 = hardwareMap.get(Servo.class, "Claw1");
        claw2 = hardwareMap.get(Servo.class, "Claw2");

        claw1.setPosition(Claw_Home);
        claw2.setPosition(Claw_Home);

    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            claw1.setPosition(0);
            claw2.setPosition(0);
        }else if (gamepad2.y) {
            claw1.setPosition(0.5);
            claw2.setPosition(0.5);
        }

    }
     */
    //


    /*                           Auto Approach program for now:
Once the robot is under the rigging, the driver will press the bumpers and using april tag detections,
The robot will approach the backstage until it is close enough for the manipulator to drop the pixels
 */
  /*     @TeleOp
    public class PreProgramEverything extends OpMode {
        IMU _orient;
        DcMotor _rightFront;
        DcMotor _leftFront;
        DcMotor _rightBack;
        DcMotor _leftBack;

        @Override
        public void init() {
            _orient = hardwareMap.get(IMU.class, "BHI260AP");
            _leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
            _rightFront = hardwareMap.get(DcMotor.class, "RightFront");
            _leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
            _rightBack = hardwareMap.get(DcMotor.class, "RightBack");

            //assumption - right side reversed
            _rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            _rightBack.setDirection(DcMotorSimple.Direction.REVERSE);





        }

        @Override
        public void loop() {
            //These have to be in the loop to be able to use the april tag detections in driver control
            AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();

            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcamera"))
                    .setCameraResolution(new Size(640,480))
                    .build();
            // variables make the math easier to type, these have to be in the loop or it won't work
            double y;
            double x;
            double rightX;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;
            //we have to initialize the variable to control speed percentage
            double speed = -0.5;

            //The camera y value should be equal to around the measured distance in inches plus 0.75
            if (gamepad1.left_bumper && gamepad1.right_bumper && tagProcessor.getDetections().size()>0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if (tag.ftcPose.y>18){
                    speed = -0.8;
                    _leftFront.setPower(speed);
                    _leftBack.setPower(speed);
                    _rightFront.setPower(speed);
                    _rightBack.setPower(speed);
                } else if (tag.ftcPose.y<18) {
                    speed = -0.25;
                    _leftFront.setPower(speed);
                    _leftBack.setPower(speed);
                    _rightFront.setPower(speed);
                    _rightBack.setPower(speed);
                } else if (tagProcessor.getDetections().size() == 0) {
                    speed = -0.5;
                    _leftFront.setPower(((y + x) + rightX) * speed);
                    _leftBack.setPower(((y - x) + rightX) * speed);
                    _rightFront.setPower(((y - x) - rightX) * speed);
                    _rightBack.setPower(((y + x) - rightX) * speed);
                }
            }
        }
    }
*/





