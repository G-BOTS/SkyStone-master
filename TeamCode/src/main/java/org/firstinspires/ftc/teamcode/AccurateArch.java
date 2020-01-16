package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
@Disabled

public class AccurateArch extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSky robot   = new HardwareSky();   // Use  Skybot hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.667;     // This is < 1.0 if geared UP for 16 to 24 tooth sprockets
    static final double     WHEEL_DIAMETER_INCHES   = 2.83 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 1;
    static final double  SPEED_ADJUST = 1; //0.585197;
    private double     GYRO_VALUE              = 1;
    private double     LPOWER                  = 1;
    private double     RPOWER                  = 1;
    private double     GYRO_TARGET             = 2;
    private double     ArcError;
    private double varyPower;

    //DigitalChannel digitalTouch;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //digitalTouch = HardwareSky.get(DigitalChannel.class, "sensor_digital");

        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        encoderDrive(DRIVE_SPEED,1,   -10, -10, 4.0);  // S1: Drive forward 4 Sec timeout
        encoderDrive(TURN_SPEED,1,   5, -5, 4.0);  // S1: Drive forward 4 Sec timeout
        encoderDrive(DRIVE_SPEED,1,   -12, -12, 4.0);  // S1: Drive forward 4 Sec timeout
        encoderDrive(DRIVE_SPEED,1,   -5, 5, 4.0);  // S1: Drive forward 4 Sec timeout
        encoderDrive(DRIVE_SPEED,1,   -10, -10, 4.0);  // S1: Drive forward 4 Sec timeout
        robot.left_hand.setPosition(0.31);
        robot.right_hand.setPosition(0.69);//hook foundation
        sleep(200);
        curveBack(1,0.2,5);//0.8 and 0.3856
        //encoderDrive(DRIVE_SPEED,1,   -25, -25, 4.0);  // S1: Drive forward 4 Sec timeout
        //robot.right_hand.setPosition(0.2);
        //robot.left_hand.setPosition(0.8);
       // encoderDrive(DRIVE_SPEED,1,   -29, -29, 4.0);  // S1: Drive forward 4 Sec timeout


        telemetry.addData("Distance"," %.2f,sensorRange.getDistance(DistanceUnit.MM)");
        telemetry.addData("Color","%.2f,sensorColor.getColor(red)");
        telemetry.addData("Touch", "%.2f, sensorTouch.getTouch(Complete");


        telemetry.update();
    }


    public void encoderDrive(double speed, double adjust, double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower((Math.abs(speed))*adjust);// adjust will cause the robot to drive in a arc

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void archDrive(double speed, double adjust, double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (GYRO_VALUE > GYRO_TARGET)   {

        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower((Math.abs(speed))*adjust);// adjust will cause the robot to drive in a arc


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }


            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   }
    }
    public void curveBack(double leftPower,double rightPower,double ArcLeft)// arcleft is the targer arc
     {
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setPower(leftPower);
        varyPower= leftPower;
        robot.rightDrive.setPower(rightPower);
        while(opModeIsActive() && (robot.leftDrive.getCurrentPosition()) < -2000){
            ArcError = GYRO_TARGET-GYRO_VALUE;
            varyPower = varyPower +(varyPower*ArcError);
            robot.leftDrive.setPower(varyPower);


        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}


