package CompCtrl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.sensorModel.StartRecordingAction;

/**
 * Creates a FRI Session.
 */
public class GraspComp extends RoboticsAPIApplication
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
	
	/*private static final double stiffnessZ = 5;  	//unit: N/m. Default: 2000
	private static final int stiffnessY = 5;
	private static final int stiffnessX = 5;*/
    private static final double StiffnessTran = 0.1;  	//unit: N/m. Default: 2000
	private static final double StiffnessRot = 0.1;  	//unit: Nm/rad  Default: 300
    
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
    	
    	//Record data
    	DataRecorder rec = new DataRecorder();
    	rec.setFileName("Recording_5.log");
    	rec.setSampleInterval(10);
    	rec.setTimeout(1000, TimeUnit.HOURS);
    	
    	
    	rec.addCartesianForce(_Gripper.getFrame("/CompCenter"), null);	//record content, orintation with Measured frame
    	rec.addCartesianTorque(_Gripper.getFrame("/CompCenter"), null);
    	rec.addCommandedCartesianPositionXYZ(_lbr.getFlange(), 
    			getApplicationData().getFrame("/BaseFrame"));
    	rec.addCurrentCartesianPositionXYZ(_lbr.getFlange(), 
    			getApplicationData().getFrame("/BaseFrame"));
    	rec.addCurrentCartesianPositionXYZ(_Gripper.getFrame("/ContPlane"), 
    			getApplicationData().getFrame("/BaseFrame"));
    	rec.addCurrentJointPosition(_lbr, AngleUnit.Degree);
    	
    	rec.enable();
    	
    	//triger start
    	/*StartRecordingAction startAction = new StartRecordingAction(rec);
    	ForceCondition startCondition =
    	ForceCondition.createSpatialForceCondition(_Gripper.getFrame("/CompCenter"),
    	1.0);*/
    	
    	//_Gripper.move(ptp(getApplicationData().getFrame("/StarGraspPnt")).setJointVelocityRel(0.25));
    	getLogger().info("Grasp Ready. Please start gripper!"); 
    	
    	
    	CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X, CartDOF.Y, CartDOF.Z).setStiffness(StiffnessTran);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(StiffnessRot);
    	impedanceControlMode.setMaxControlForce(MaxForceTCP, MaxForceTCP, MaxForceTCP, MaxTorqueTCP, MaxTorqueTCP, MaxTorqueTCP, true);
        
    	rec.startRecording();
    	
        getLogger().info("start Postionhold.");       
        PositionHold posHold = new PositionHold(impedanceControlMode, 60, TimeUnit.DAYS);
        _Gripper.getFrame("/CompCenter").move(posHold);
        
        rec.stopRecording();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final GraspComp app = new GraspComp();
        app.runApplication();
    }

}
