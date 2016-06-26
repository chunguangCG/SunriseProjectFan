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

/**
 * Creates a FRI Session.
 */
public class SwitchPos extends RoboticsAPIApplication
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
    private static final double StiffnessTran = 1000;  	//unit: N/m. Default: 2000
	private static final double StiffnessRot = 150;  	//unit: Nm/rad  Default: 300
    
	private static final double MaxForceTCP = 0.1;		//unit: N
	private static final double MaxTorqueTCP = 0.1;  	//unit: Nm


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
    	
    	_Gripper.move(ptp(getApplicationData().getFrame("/GraspDonePnt")).setJointVelocityRel(0.25));
    	getLogger().info("Ready to pull up"); 
    	
    	
    	CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X, CartDOF.Y, CartDOF.Z).setStiffness(StiffnessTran);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(StiffnessRot);
    	impedanceControlMode.setMaxControlForce(MaxForceTCP, MaxForceTCP, MaxForceTCP, MaxTorqueTCP, MaxTorqueTCP, MaxTorqueTCP, true);
               
        /*getLogger().info("start Postionhold.");       
        PositionHold posHold = new PositionHold(impedanceControlMode, 60, TimeUnit.DAYS);
        _Gripper.getFrame("/CompCenter").move(posHold);*/
        
    	_Gripper.move(linRel(Transformation.ofDeg(0,0,100,0,0,0),getApplicationData().getFrame("/BaseFrame")));
    	getLogger().info("Pull up OK.");
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final SwitchPos app = new SwitchPos();
        app.runApplication();
    }

}
