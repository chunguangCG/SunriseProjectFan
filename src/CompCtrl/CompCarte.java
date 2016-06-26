package CompCtrl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Creates a FRI Session.
 */
public class CompCarte extends RoboticsAPIApplication
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
	
	private static final int stiffnessZ = 5;  	//unit: N/m. Default: 2000
	private static final int stiffnessY = 5;
	private static final int stiffnessX = 5;
	
	private static final int StiffnessRot = 1;  	//unit: Nm/rad  Default: 300
    



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
    	
    	_Gripper.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.25));
    	getLogger().info("Gripper frame to P1."); 
    	
    	_Gripper.getFrame("/CompCenter").move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(150.0));
    	getLogger().info("CompCenter frame to P1. Now START compliance");
    		
    	// move to start pose
    	//_Gripper.move(ptp(0., Math.toRadians(30), .0, -Math.toRadians(60), .0, Math.toRadians(90), .0).setJointVelocityRel(0.5)); 	
    	
    	CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(StiffnessRot);
		
               
        /*getLogger().info("start Postionhold.");       
        PositionHold posHold = new PositionHold(impedanceControlMode, 60, TimeUnit.DAYS);
        _Gripper.getFrame("/CompCenter").move(posHold);*/
        
    	_Gripper.getFrame("/CompCenter").move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(150.0).setMode(impedanceControlMode));
    	getLogger().info("CompCenter frame to P2.");
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final CompCarte app = new CompCarte();
        app.runApplication();
    }

}
