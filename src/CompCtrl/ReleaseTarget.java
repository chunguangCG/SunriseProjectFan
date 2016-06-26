package CompCtrl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Creates a FRI Session.
 */
public class ReleaseTarget extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private Tool _Gripper;
    
    //Impedance para.
    /*private static final int stiffnessZ = 0;  	//unit: N/m. Default: 2000
	private static final int stiffnessY = 0;
	private static final int stiffnessX = 0;
	
	private static final int StiffnessRot = 0;  	//unit: Nm/rad  Default: 300*/
    private static final double StiffnessTran = 3000;  	//unit: N/m. Default: 2000
	private static final double StiffnessRot = 300;  	//unit: Nm/rad  Default: 300
    
	private static final double MaxForceTCP = 0.1;		//unit: N
	private static final double MaxTorqueTCP = 0.1;  	//unit: Nm
	
	private final static String informationText="Is gripper safe?";

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        
        //copy tool information "Gripper" to controller
        _Gripper = getApplicationData().createFromTemplate("Gripper");
    }

    @Override
    public void run()
    {
    	//use tool
    	_Gripper.attachTo(_lbr.getFlange());
   	
    	CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		//impedanceControlMode.parametrize(CartDOF.X, CartDOF.Y, CartDOF.Z).setStiffness(StiffnessTran);
		//impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(StiffnessRot);
    	impedanceControlMode.setMaxControlForce(MaxForceTCP, MaxForceTCP, MaxForceTCP, MaxTorqueTCP, MaxTorqueTCP, MaxTorqueTCP, true);  
        
    	//_Gripper.move(ptp(getApplicationData().getFrame("/GraspDonePnt")).setJointVelocityRel(0.2).setMode(impedanceControlMode));
    	
    	getLogger().info("Ready to release"); 
    	
    	_Gripper.move(linRel(Transformation.ofDeg(0,0,100,0,0,0),getApplicationData().getFrame("/BaseFrame")).setCartVelocity(100.0).setMode(impedanceControlMode));
    	getLogger().info("Release target OK.");
    	
    	int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "Yes", "No");
        while (isCancel == 1)
        {
        	_Gripper.move(linRel(Transformation.ofDeg(0,0,5,0,0,0),getApplicationData().getFrame("/BaseFrame")).setCartVelocity(100.0).setMode(impedanceControlMode));
        	//getLogger().info("Release target OK.");
        	isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "Yes", "No");
        	if (isCancel == 0)
        	{
        		break;
        	}
        	//return;
        }
    	
    	_Gripper.move(ptp(getApplicationData().getFrame("/GuideStarPnt")).setJointVelocityRel(0.2));
    	getLogger().info("Back to GuidePnt again.");
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final ReleaseTarget app = new ReleaseTarget();
        app.runApplication();
    }

}
