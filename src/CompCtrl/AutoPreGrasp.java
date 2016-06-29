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
import com.kuka.roboticsAPI.geometricModel.Workpiece;
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
public class AutoPreGrasp extends RoboticsAPIApplication
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
	private static final double StiffnessRot = 300;  	//unit: Nm/rad  Default: 200
    
	private static final double MaxForceTCP = 8;		//unit: N
	private static final double MaxTorqueTCP = 1.5;  	//unit: Nm
	
	private final static String informationText="Line 2 100mm above StarGraspPnt safe?";
	
	private final static String informationCheckGripper="Gripper Opened & Path safe?";

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
        
    	PositionHold posHold = new PositionHold(impedanceControlMode, 1, TimeUnit.SECONDS);
    	
    	//move to the above
    	int PathSafeAbove = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "Yes", "No");
        if (PathSafeAbove == 1)
        {
            return;
        }
    	_Gripper.move(lin(getApplicationData().getFrame("/StarGraspPnt/AbovePnt")).setCartVelocity(100.0).setMode(impedanceControlMode));
    	
    	
    	
    	int PathSafe = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationCheckGripper, "Yes", "No");
        while (PathSafe == 1)
        {
        	
        	_Gripper.move(posHold);

        	PathSafe = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationCheckGripper, "Yes", "No");
        	if (PathSafe == 0)
        	{
        		break;
        	}
        }
    	

    	//_MultiFunTool.move(linRel(Transformation.ofDeg(0,0,100,0,0,0),getApplicationData().getFrame("/BaseFrame")).setCartVelocity(100.0).setMode(impedanceControlMode));
    	_Gripper.move(lin(getApplicationData().getFrame("/StarGraspPnt")).setCartVelocity(100.0).setMode(impedanceControlMode));
    	
        //getLogger().info("start Postionhold.");       
        PositionHold posHoldLong = new PositionHold(impedanceControlMode, 60, TimeUnit.DAYS);
        _Gripper.move(posHoldLong);
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final AutoPreGrasp app = new AutoPreGrasp();
        app.runApplication();
    }

}
