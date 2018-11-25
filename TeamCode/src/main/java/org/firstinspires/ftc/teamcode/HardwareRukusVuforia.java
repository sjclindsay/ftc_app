package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class HardwareRukusVuforia {
    private double[][] VuforiaCoords = {{0, 0, 0},
            {0, 0, 0}};
    public OpenGLMatrix lastLocation = null;
    private VuforiaTrackable currentTrackable;
    private VectorF translation = null;
    private Orientation rotation = null;
    private boolean coordinatesNotUpdated = true;

    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;
    public RelicRecoveryVuMark vuMark = null;

    public enum vuForiaCoord {
        tX,
        tY,
        tZ,
        rX,
        rY,
        rZ
    }

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    private boolean targetVisible = false;
    private boolean webCamConnected = false;

    private WebcamName webcamName;
    private String webCamDeviceName;

    public  HardwareRukusVuforia(){
        webcamName = null;
    }

    public HardwareRukusVuforia(String Camera) {
        webcamName = null;
        webCamConnected = true;
        webCamDeviceName = Camera;
    }

    public VuforiaLocalizer vuforia;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackables targetsRoverRuckus = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

        webcamName = ahwMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AdN8DU3/////AAABmQ/dUsRwkUmdhO1NgtnuZVw0CylAO1tY1lqGEWdg5a5+41KHLK3UJVHmeOGom8bxbonz72IVRobln4H9HhE6Sb+N1CLkGDZXFJGo2SqKmuizaBbxks84g/7b8nazNwZe8SB++OJCbx4zJKHduNQFOr+BRa4PLP+vsRdyat7xfrsVCVhSqy2PDmTFWV350G+mMtF3NXvpZYPlDVP5LU7cdKTkHUfgGPYYuw8X/ryNA+OWnL+d17S7zMbjGeyzHuV5+BnPq5AqdS6QLRpWQtkO6m0cRub5hTiJOSFkzb+rcZG+ZODf5976lST5P401gxp6pN3BJBvS3hOEMURJolf8OcEOkM/RA+fqUzGxpLDnCblW";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the Front (LowRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        if (webCamConnected) {
            parameters.cameraName = webcamName;
        }
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 152;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 356;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 152;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 90));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }


    }
        //telemetry.addData(">", "Press Play to start");
        //telemetry.update();
        //waitForStart();
    public void start () {

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public Boolean isTargetVisible () {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        return(targetVisible);
    }

    public VectorF getTranslation() {
        translation = lastLocation.getTranslation();
        return translation;
    }

    public Orientation getRotation() {
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        return rotation;
    }

    public void UpdateLocation () {
        coordinatesNotUpdated = true;
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;
                currentTrackable = trackable;
                RobotLog.i("UpdateLocation: isVisible true Current trackable "+currentTrackable.getName());
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            if (targetVisible && coordinatesNotUpdated) {
                RobotLog.i("UpdateLocation: translation Trackable is visible");
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                // express the rotation of the robot in degrees.
                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = translation.get(0);
                double tY = translation.get(1);
                double tZ = translation.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rotation.firstAngle;
                double rY = rotation.secondAngle;
                double rZ = rotation.thirdAngle;

                VuforiaCoords [0] [0] = tX ;
                VuforiaCoords [0] [1] = tY ;
                VuforiaCoords [0] [2] = tZ ;

                VuforiaCoords [1] [0] = rX ;
                VuforiaCoords [1] [1] = rY ;
                VuforiaCoords [1] [2] = rZ ;

                RobotLog.i("Vuforia Rotation: Z " + rotation.thirdAngle +
                        ", Y " + rotation.secondAngle +
                        ", X " + rotation.firstAngle );
                RobotLog.i("Vuforia Translation: Z " + translation.get (2) +
                        ", Y " + translation.get (1) +
                        ", X " + translation.get(0));

                coordinatesNotUpdated = false;
            }
        }

    }

    public  java.lang.String getTrackableName() {
        return(currentTrackable.getName());
    }

    public void updateVuforiaCoords(OpenGLMatrix pose){

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */

      if (pose != null) {
          VectorF trans = pose.getTranslation();
          Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

          // Extract the X, Y, and Z components of the offset of the target relative to the robot
          double tX = trans.get(0);
          double tY = trans.get(1);
          double tZ = trans.get(2);

          // Extract the rotational components of the target relative to the robot
          double rX = rot.firstAngle;
          double rY = rot.secondAngle;
          double rZ = rot.thirdAngle;

          VuforiaCoords [0] [0] = tX ;
          VuforiaCoords [0] [1] = tY ;
          VuforiaCoords [0] [2] = tZ ;

          VuforiaCoords [1] [0] = rX ;
          VuforiaCoords [1] [1] = rY ;
          VuforiaCoords [1] [2] = rZ ;
          RobotLog.i("Vuforia Rotation: Z " + rot.thirdAngle +
                  ", Y " + rot.secondAngle +
                  ", X " + rot.firstAngle );
          RobotLog.i("Vuforia Translation: Z " + trans.get (2) +
                  ", Y " + trans.get (1) +
                  ", X " + trans.get(0));

      }
  }

  public double getVuforiaCoords (vuForiaCoord axis) {
      double outPut = 0 ;

      if (axis == vuForiaCoord.tX) {outPut = VuforiaCoords [0][0] ;}
      if (axis == vuForiaCoord.tY) {outPut = VuforiaCoords [0][1] ;}
      if (axis == vuForiaCoord.tZ) {outPut = VuforiaCoords [0][2] ;}

      if (axis == vuForiaCoord.rX) {outPut = VuforiaCoords [1][0] ;}
      if (axis == vuForiaCoord.rY) {outPut = VuforiaCoords [1][1] ;}
      if (axis == vuForiaCoord.rZ) {outPut = VuforiaCoords [1][2] ;}

      return  outPut ;
  }






            //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */


                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */

              //  OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */

                //if (pose != null) {
                  //  VectorF trans = pose.getTranslation();
                    //Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                /*    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }

    */

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                UpdateLocation();
            }
        });
        if (targetVisible) {
            telemetry.addLine()
                    .addData("Visible Target", currentTrackable.getName());
            telemetry.addLine()
                    .addData("X Vuforia ", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.tX));
                        }
                    })
                    .addData("X Rot Vuforia ", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.rX));
                        }
                    })
                    .addData("Y Vuforia ", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.tY));
                        }
                    })
                    .addData("Y Rot Vuforia ", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.rY));
                        }
                    }).addData("Z Vuforia ", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.tZ));
                }
            }).addData("Z Rot Vuforia ", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.rZ));
                }
            });
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
    }
}
