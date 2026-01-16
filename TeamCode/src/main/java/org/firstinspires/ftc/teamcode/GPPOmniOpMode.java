package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;


@TeleOp(name = "21 GPP - 2026 Omni Op Mode OBJ")
public class GPPOmniOpMode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor magazine;
    private DcMotor outtake;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    private PredominantColorProcessor colorSensor;

    //VisionPortal.Builder myVisionPortalBuilder;
    //VisionPortal.Builder secondPortalBuilder;
    boolean USE_WEBCAM_1;
    //boolean MagazinePositiveMotion;
    int Portal_1_View_ID;
    int counter = 0;
    boolean USE_WEBCAM_2;
    int Portal_2_View_ID;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final float COUNTS_PER_FULL_REV = 96.245f;    // This should be 47.1 at some point but it's fine for now
    float floatTargetPosition = 0;
    //static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    //static final double     TURN_SPEED              = 0.5;

    String[] colors = new String[3];
    float slotNumber = 0f;
    float topSlotNumber = 1.5f;
    int magazinePos;
    String swatchResult;
    float outtakePower = 0f;

    String aprilTagID = String.valueOf("21");
    int currentMagazinePosition = 0;

    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();





    @Override
    public void runOpMode() {
        boolean MagazinePositiveMotion = false;
        ElapsedTime runtime;
        boolean BottomAligned;
        float axial;
        float lateral;
        float yaw;
        double driveSpeed;
        boolean PDIcontroller;
        double max;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        magazine = hardwareMap.get(DcMotor.class, "magazine");
        outtake = hardwareMap.get(DcMotor.class, "outtake");

        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        magazine.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magazinePos = 0;
        BottomAligned = true;
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //USE_WEBCAM_1 = false;
        //USE_WEBCAM_2 = true;
        //initMultiPortals();///INIT YOUR ONE PORTAL

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        //PredominantColorProcessor.Swatch.RED,
                        //PredominantColorProcessor.Swatch.BLUE,
                        //PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();
        runtime.reset();

        magazine = (DcMotorEx) hardwareMap.get(DcMotor.class, "magazine");
        //controller = new PIDController(p, i, d);  // ← ADD THIS LINE HERE

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        magazine.setDirection(DcMotor.Direction.FORWARD);

        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        ///magazine.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", magazine.getCurrentPosition());
        telemetry.update();
        // Wait for the game to start (driver presses START)

        // This OpMode shows AprilTag recognition and pose estimation.

        // Initialize AprilTag before waitForStart.

        /*colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.4, 0.4, 0.4, -0.4))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.WHITE,
                        //PredominantColorProcessor.Swatch.BLUE,
                        //PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK)
                //PredominantColorProcessor.Swatch.WHITE)
                .build(); */


        //telemetry.addData("Webcam 1 available", hardwareMap.get(WebcamName.class, "Webcam 1") != null);
        telemetry.addData("Webcam 1 available", hardwareMap.get(WebcamName.class, "Webcam 1") != null);
        telemetry.update();

        //initAprilTag();


        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        //VisionPortal portal = new VisionPortal.Builder()
        //.addProcessor(colorSensor);
        //.setCameraResolution(new Size(320, 240))
        //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //.build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        // Run until the end of the match (driver presses STOP)
        try {
            while (opModeIsActive()) {
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);
                // Start Section: Drive
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                // Note: pushing stick forward gives negative value
                axial = -gamepad1.left_stick_y;
                lateral = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                if (gamepad1.left_bumper) {
                    driveSpeed = 0.5;
                } else {
                    driveSpeed = 1;
                }
                if (gamepad2.y) {
                    intake.setPower(1);
                } else {
                    if (gamepad2.x) {
                        intake.setPower(-0.4);
                    } else {
                        intake.setPower(0);
                    }
                }

                if (gamepad2.aWasPressed()) {
                    if (outtakePower != 0.6f) {
                        outtakePower = 0.6f;
                    }
                    if (outtakePower == 0.6f) {
                        outtakePower = 1.0f;
                    }
                }
                if (gamepad2.b) {
                    outtakePower = 0f;
                }
                if (gamepad2.back) {
                    outtakePower = -0.3f;
                }

                outtake.setPower(outtakePower);

                //PredominantColorProcessor.Result result = colorSensor.getAnalysis();
                //swatchResult = result.closestSwatch.toString();
                //telemetry.addData("Color Scanned", swatchResult);

                if (left_bumper_was_pressed()) { //control for autosort
                    PredominantColorProcessor.Result result2 = colorSensor.getAnalysis();
                    swatchResult = result2.closestSwatch.toString();
                    if ((((swatchResult.equals("ARTIFACT_PURPLE")) || (swatchResult.equals("ARTIFACT_GREEN"))))) {
                        fullRotation(1, 1, true, swatchResult);
                    } else {
                        telemetry.addData("Color Scanned", swatchResult);
                        telemetry.addData("Did not rotate", "No color detected");
                        //telemetry.update();
                    }
                }

                if (dpad_up_was_pressed()) {
                    fullRotation(1, 1, false, null);
                    MagazinePositiveMotion = true;
                } else {
                    if (dpad_down_was_pressed()) {
                        fullRotation(-1, 1, false, null);
                        MagazinePositiveMotion = false;
                    } else {
                        if (gamepad2.left_stick_button) {
                            MagazinePositiveMotion = true;
                            magazine.setPower(0.7);
                            magazinePos = magazine.getCurrentPosition();
                            //BottomAligned = null;
                        } else {
                            if (gamepad2.right_stick_button) {
                                MagazinePositiveMotion = false;
                                magazine.setPower(-0.7);
                                magazinePos = magazine.getCurrentPosition();
                            } else {
                                if (dpad_right_was_pressed()) {
                                    MagazinePositiveMotion = true;
                                    fullRotation(0.5f, 1, false, "null");
                                /*if (BottomAligned == false) {
                                    BottomAligned = true;
                                } else if (BottomAligned == true) {
                                    BottomAligned = false;
                                } */
                                } else {
                                    if (dpad_left_was_pressed()) {
                                        MagazinePositiveMotion = false;
                                        fullRotation(-0.5f, 1, false, "null");
                                    /*if (BottomAligned == false) {
                                        BottomAligned = true;
                                    } else if (BottomAligned == true) {
                                        BottomAligned = false;
                                    }*/
                                    } else {
                                        //PDIcontroller = false;
                                    }
                                }
                            }
                        }
                    }
                }
                if (right_trigger_was_pressed()) {
                    if (counter == 0) { //On the first rotation, align
                        if (colors[0] != null && colors[1] != null && colors[2] != null) {
                            fullRotation(0.5f, 1f, false, "null");
                        }
                    } else {
                        telemetry.addData("magazine slot", "empty");
                        telemetry.update();
                    }

                    if (counter >= 3) { //On the 4th button press, DO NOT LAUNCH and realign
                        fullRotation(0.5f, 1f, false, "null");
                        counter = 0;
                    } else {
                        launchMotif(21, counter); ///NEED TO CALL LAUNCHMOTIF THREE TIMES BASED ON WHICH ONE WAS ACTUALLY LAUNCHED
                        counter += 1;
                    }

                }
                //}

                /// move the magazine with custom "PID"
                if (magazinePos - magazine.getCurrentPosition() >= 11 && MagazinePositiveMotion == true) {
                    magazine.setPower(1);
                    PDIcontroller = false;
                } else {
                    if (magazinePos - magazine.getCurrentPosition() <= -11 && MagazinePositiveMotion == false) {
                        magazine.setPower(-1);
                        PDIcontroller = false;
                    } else {
                        magazine.setPower(0);
                        PDIcontroller = true;
                    }
                }
                if (PDIcontroller == true) {
                    if (magazinePos - magazine.getCurrentPosition() >= 1 && MagazinePositiveMotion == false) {
                        magazine.setPower(0.5);
                    } else {
                        if (magazinePos - magazine.getCurrentPosition() <= -1 && MagazinePositiveMotion == true) {
                            magazine.setPower(-0.5);
                        } else {
                            if (magazinePos - magazine.getCurrentPosition() >= 1 && MagazinePositiveMotion == true) {
                                magazine.setPower(0.2);
                            } else {
                                if (magazinePos - magazine.getCurrentPosition() <= -1 && MagazinePositiveMotion == false) {
                                    magazine.setPower(-0.2);
                                } else {
                                    magazine.setPower(0);
                                    PDIcontroller = false;
                                }
                            }
                        }
                    }
                }
                leftFrontPower = (axial + lateral + yaw) * driveSpeed;
                rightFrontPower = ((axial - lateral) - yaw) * driveSpeed;
                leftBackPower = ((axial - lateral) + yaw) * driveSpeed;
                rightBackPower = ((axial + lateral) - yaw) * driveSpeed;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
                if (max > 1) {
                    leftFrontPower = leftFrontPower / max;
                    rightFrontPower = rightFrontPower / max;
                    leftBackPower = leftBackPower / max;
                    rightBackPower = rightBackPower / max;
                }
                // Send calculated power to wheels.
                frontLeft.setPower(leftFrontPower);
                frontRight.setPower(rightFrontPower);
                backLeft.setPower(leftBackPower);
                backRight.setPower(rightBackPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("Status", "MagazinePos: " + magazinePos);
                telemetry.addData("Status", "CurrentPos: " + magazine.getCurrentPosition());
                telemetry.addData("Status", "Bottom Aligned: " + BottomAligned);
                //telemetry.addData("Status", "Magazine Positive Motion" + MagazinePositiveMotion);
                telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
                telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
                telemetry.addData("colors 0", colors[0]);
                telemetry.addData("colors 1", colors[1]);
                telemetry.addData("colors 2", colors[2]);
                telemetry.addData("Counter", counter);
                telemetry.addData("Output Power: ", outtakePower);



                telemetry.update();
                magazine.setTargetPosition(magazinePos);

            }
        }
        finally { //don't crash when stop
            // This ALWAYS runs, even if STOP is pressed
            try {
                if (portal != null) {
                    portal.close();
                }
            } catch (Exception e) {
                // Ignore errors closing portal 1
            }

            // Stop all motors
            try {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                intake.setPower(0);
                magazine.setPower(0);
                outtake.setPower(0);
            } catch (Exception e) {
                // Ignore errors stopping motors
            }
        }
        }


    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections () {
        leftFrontPower = gamepad1.x ? 1 : 0;
        leftBackPower = gamepad1.a ? 1 : 0;
        rightFrontPower = gamepad1.y ? 1 : 0;
        rightBackPower = false ? 1 : 0;
    }
    public void fullRotation ( float numOfRotations, float magazinePower, boolean assignNewColor, String colorResult)
    {
        if (!opModeIsActive()) return;

        if (assignNewColor) {
            //PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            if (slotNumber % 1.0 == 0) {
                colors[(int) slotNumber] = colorResult;
                if (!colors[(int) slotNumber].equals("ARTIFACT_PURPLE") && !colors[(int) slotNumber].equals("ARTIFACT_GREEN")) {
                    //colors[(int) slotNumber] = "ARTIFACT_PURPLE";///hold for error analysis
                }
            }
        }
        slotNumber += (1 * numOfRotations);
        if (slotNumber >= 3) {
            slotNumber = slotNumber - 3;
        }

        topSlotNumber += (1 * numOfRotations);
        if (topSlotNumber >= 3) {
            topSlotNumber = topSlotNumber - 3;
        }

        //Make motor move with custom PID controller -- This was written by Claude because I don't understand PID controllers

        //double power = 0;
        floatTargetPosition = floatTargetPosition + (COUNTS_PER_FULL_REV * numOfRotations);
        magazinePos = (int) floatTargetPosition;
        //magazine.setTargetPosition((int) floatTargetPosition);
    }

    //magazine.setPower(1.0);
        /*while (Math.abs(floatTargetPosition - magazine.getCurrentPosition()) > 5) { // 5 is tolerance

            double command = control.update((int) floatTargetPosition, magazine.getCurrentPosition());
            command = Math.max(-1.0, Math.min(1.0, command));
            // assign motor the PID output
            //magazine.setPower(command);
        } */


    public void launchMotif ( int aprilTagID, int ballNumber){
        float theoreticalSlot;
        float numRotationsRequired;
        String[] motif = new String[3];
        switch (aprilTagID) {
            case 21:
                motif[0] = "ARTIFACT_GREEN";
                motif[1] = "ARTIFACT_PURPLE";
                motif[2] = "ARTIFACT_PURPLE";
                break;
            case 22:
                motif[0] = "ARTIFACT_PURPLE";
                motif[1] = "ARTIFACT_GREEN";
                motif[2] = "ARTIFACT_PURPLE";
                break;
            case 23:
                motif[0] = "ARTIFACT_PURPLE";
                motif[1] = "ARTIFACT_PURPLE";
                motif[2] = "ARTIFACT_GREEN";
                break;
            default:
                telemetry.addData("APRILTAG ERROR", "no case");
                telemetry.update();
                sleep(500);
                // code block to execute if no case matches (optional)
        }

        //for (int i = 0; i < 3; i++) {
        theoreticalSlot = topSlotNumber;
        numRotationsRequired = 0f;
        /*if ((colors[(int) theoreticalSlot] == null || colors[(int) theoreticalSlot].isEmpty())) {
            telemetry.addData("value in array is empty", "ball skipped and counter reset");
            telemetry.update();
            fullRotation(0.5f, 1, false,null);
            counter = 0;
            sleep(500);
            return;
        }*/
        //if ((colors[(int) theoreticalSlot]) != (null)) {
            while ((opModeIsActive()) && (!colors[(int) theoreticalSlot].equals(motif[ballNumber]))) {
                numRotationsRequired += 1f;
                theoreticalSlot += 1f;
                if (theoreticalSlot >= 3f) {
                    theoreticalSlot = theoreticalSlot - 3f;
                }
                if (numRotationsRequired >= 3f) {
                    numRotationsRequired -= 3f;
                }
                if (numRotationsRequired == 2f) {
                    numRotationsRequired = -1f;
                }
                telemetry.addData(colors[(int) theoreticalSlot], motif[ballNumber]);
                //telemetry.update();
            }
        //}
            /*if (numRotationsRequired == 2f) { //this makes it rotate in the other direction instead of rotating twice
                numRotationsRequired = -1f;
            }*/
        telemetry.addData("numRotationsRequired", numRotationsRequired);
        telemetry.addData("launching", ballNumber);
        telemetry.addData("Bottom Slot", slotNumber);
        telemetry.addData("Top Slot", topSlotNumber);
        telemetry.addData("current color", colors[(int) theoreticalSlot]);
        telemetry.addData("Current motif", motif[ballNumber]);
        telemetry.addData("colors 0", colors[0]);
        telemetry.addData("colors 1", colors[1]);
        telemetry.addData("colors 2", colors[2]);
        telemetry.update();
        //telemetry.update();

        colors[(int) theoreticalSlot] = "";
        fullRotation(numRotationsRequired, 1F, false, "none");
        //while (magazine.isBusy() && opModeIsActive()) {
        //    telemetry.addData("status", "moving");
        //}

        //sleep(2000);//launch
        //}

    }

    //}


    // Stop all motion;
    //leftDrive.setPower(0);
    //rightDrive.setPower(0);

    // Turn off RUN_TO_POSITION
    //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //sleep(250);   // optional pause after each move.
//*******************************************
//Taken from the TwoVisionPortals.java file. Not sure what these mean but these are just methods
//*******************************************

    //private void initAprilTag() {
        //AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        // First, create an AprilTagProcessor.Builder.
        //myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create each AprilTagProcessor by calling build.
        //myAprilTagProcessor_1 = myAprilTagProcessorBuilder.build();
        //myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
        //Make_first_VisionPortal();
        //Make_second_VisionPortal();
    //}

    /**
     * Describe this function...
     */
    /*private void Make_first_VisionPortal() {
        // Create a VisionPortal.Builder and set attributes related to the first camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM_1) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor_1);
        // Add the Portal View ID to the VisionPortal.Builder
        // Set the camera monitor view id.
        myVisionPortalBuilder.setLiveViewContainerId(Portal_1_View_ID);
        // Create a VisionPortal by calling build.
        myVisionPortal_1 = myVisionPortalBuilder.build();
    }*/

    /**
     * Describe this function...
     */

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     *
     * @return
     */
    /*private String AprilTag_telemetry_for_Portal_1() {
        List<AprilTagDetection> myAprilTagDetections_1;
        AprilTagDetection thisDetection_1;

        // Get a list of AprilTag detections.
        myAprilTagDetections_1 = myAprilTagProcessor_1.getDetections();
        telemetry.addData("Portal 1 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_1));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection thisDetection_1_item : myAprilTagDetections_1) {
            thisDetection_1 = thisDetection_1_item;
            telemetry.addLine("First Apriltag ID (if 1+ detected, will be SKIPPED" + thisDetection_1.id);
            if (thisDetection_1.metadata != null) {  // Check first!
                return String.valueOf(thisDetection_1.id);
            }
            // Display info about the detection.
            //telemetry.addLine("");
        /* if (thisDetection_1.metadata != null) {
            telemetry.addLine("==== (ID " + thisDetection_1.id + ") " + thisDetection_1.metadata.name);
            //telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_1.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.z, 6, 1) + "  (inch)");
            //telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_1.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.roll, 6, 1) + "  (deg)");
            //telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_1.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        } else {
            //telemetry.addLine("==== (ID " + thisDetection_1.id + ") Unknown");
            return null; */
            //telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_1.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_1.center.y, 6, 0) + " (pixels)");
        //}
        //return null;
    //}

//}

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    /**
     * Describe this function...
     */

    private boolean dpad_up_was_pressed() {
        return currentGamepad2.dpad_up && !previousGamepad2.dpad_up;
    }
    private boolean dpad_left_was_pressed() {
        return currentGamepad2.dpad_left && !previousGamepad2.dpad_left;
    }
    private boolean dpad_right_was_pressed() {
        return currentGamepad2.dpad_right && !previousGamepad2.dpad_right;
    }
    private boolean right_trigger_was_pressed() {
        return currentGamepad2.right_bumper && !previousGamepad2.right_bumper;
    }
    private boolean left_bumper_was_pressed() {
        return currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
    }
    private boolean dpad_down_was_pressed() {
        return currentGamepad2.dpad_down && !previousGamepad2.dpad_down;
    }
}




