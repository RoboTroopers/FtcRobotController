package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.AutoRingDetection.State.*;

@Autonomous(name = "Ring Detector Auto", group="Autonomous")
public class AutoRingDetection extends AutoBase
{
    
    protected enum State
    {
        START,
        DETECT,
        NO_RING_POS,
        SINGLE_RING_POS,
        QUAD_RING_POS,
    }

    private final boolean redAlliance = true;
    private boolean doDetection = true;


    @Override
    protected int handleState(int currentState, int seconds)
    {
        State state = getCurrentState(State.values());
        telemetry.addData("Current state", state);

        switch (getCurrentState(State.values()))
        {
            case START:
                return DETECT.ordinal();
            //break;

            case DETECT:
                if (seconds > 1.5)
                {
                    return getDetectedRingAmountPos().ordinal();
                }
                break;
                
            case NO_RING_POS:
                break;

            case SINGLE_RING_POS:
                break;
                
            case QUAD_RING_POS:
                break;
        }
        return currentState;
    }
    
    
    
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_SINGLE_ELEMENT = "Single";
    private static final String LABEL_QUAD_ELEMENT = "Quad";

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
    private static final String VUFORIA_KEY =
            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void init()
    {
        super.init();
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null)
        {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
    
    
    protected State getDetectedRingAmountPos()
    {
        State ringAmount = NO_RING_POS; // Default to no rings if no recognitions are made.
        
        if (tfod != null)
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (doDetection && updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // Step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    
                    if (recognition.getLabel().equals(LABEL_SINGLE_ELEMENT))
                    {
                        ringAmount = SINGLE_RING_POS;
                    }
                    else if (recognition.getLabel().equals(LABEL_QUAD_ELEMENT))
                    {
                        ringAmount = QUAD_RING_POS;
                    }
                }
            }
        }
        
        return ringAmount;
    }

    @Override
    public void stop()
    {
        if (tfod != null)
        {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

}
