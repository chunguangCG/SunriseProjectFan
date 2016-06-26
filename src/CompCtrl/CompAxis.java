package CompCtrl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Creates a FRI Session.
 */
public class CompAxis extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private Tool _Gripper;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        
        //copy tool information "Gripper" to controller
        _Gripper = getApplicationData().createFromTemplate("Gripper");
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.168.1.13";
    }

    @Override
    public void run()
    {
    	//use tool
    	_Gripper.attachTo(_lbr.getFlange());

        // move to start pose
    	_Gripper.move(ptp(0., Math.toRadians(30), .0, -Math.toRadians(60), .0, Math.toRadians(90), .0).setJointVelocityRel(0.5));
        JointImpedanceControlMode _jntImp = new JointImpedanceControlMode(7); 
        _jntImp.setStiffnessForAllJoints(0).setDampingForAllJoints(1.0);
        
       
        getLogger().info("start Postionhold.");
        
        //PositionHold posHold = new PositionHold(new PositionControlMode(), 60, TimeUnit.DAYS);
        PositionHold posHold = new PositionHold(_jntImp, 60, TimeUnit.DAYS);
        _Gripper.move(posHold);
        //_lbr.move(posHold.addMotionOverlay(jointOverlay));
        
        
        // done
 
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final CompAxis app = new CompAxis();
        app.runApplication();
    }

}
