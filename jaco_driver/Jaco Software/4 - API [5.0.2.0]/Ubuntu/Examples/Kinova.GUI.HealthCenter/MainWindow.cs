using System;
using Gtk;
using Kinova.DLL.SafeGate;
using Kinova.API.Jaco;
using System.Threading;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.Data.Jaco.Diagnostic;

public partial class MainWindow : Gtk.Window
{
	CJacoArm jaco;
	
    Thread m_WorkThread;
	
	//Delegate method for the thread
    private delegate void DoWorkDelegate();
    ManualResetEvent mre = new ManualResetEvent(false);
	
	CPosition position = new CPosition();
	
	bool m_Run = true;
	
	Fixed m_MainPanel;
	Label m_OutConnectionStatus;
	
	Label m_MainTitle;
	Gdk.Color m_ColorConnectionClosed;
	Gdk.Color m_ColorConnectionOpened;
	
	Label m_TxtTimeAbsolute;
	Label m_TxtTimeStartUp;
	Label m_TxtTimeStampSavings;
	Label m_TxtSupplyVoltage;
	Label m_TxtCurrentConsumed;
	Label m_TxtPowerConsumed;
	Label m_TxtAveragePower;
	Label m_TxtAccelerationX;
	Label m_TxtAccelerationY;
	Label m_TxtAccelerationZ;
	Label m_TxtCodeVersion;
	Label m_TxtCodeRevision;
	Label m_TxtControlOperator;
	Label m_TxtControlMode;
	Label m_TxtHandMode;
	Label m_TxtConnectedJointQuantity;
	Label m_TxtPositionType;
	Label m_TxtErrorsSpiMain;
	Label m_TxtErrorsSpiExternal;
	Label m_TxtErrorsMainCAN;
	Label m_TxtErrorsExternalCAN;
	Label m_TxtCartesianX;
	Label m_TxtCartesianY;
	Label m_TxtCartesianZ;
	Label m_TxtCartesianThetaX;
	Label m_TxtCartesianThetaY;
	Label m_TxtCartesianThetaZ;
	Label m_TxtAngle1;
	Label m_TxtAngle2;
	Label m_TxtAngle3;
	Label m_TxtAngle4;
	Label m_TxtAngle5;
	Label m_TxtAngle6;
	
	Label m_OutTimeAbsolute;
	Label m_OutTimeStartUp;
	Label m_OutTimeStampSavings;
	Label m_OutSupplyVoltage;
	Label m_OutCurrentConsumed;
	Label m_OutPowerConsumed;
	Label m_OutAveragePower;
	Label m_OutAccelerationX;
	Label m_OutAccelerationY;
	Label m_OutAccelerationZ;
	Label m_OutCodeVersion;
	Label m_OutCodeRevision;
	Label m_OutControlOperator;
	Label m_OutControlMode;
	Label m_OutHandMode;
	Label m_OutConnectedJointQuantity;
	Label m_OutPositionType;
	Label m_OutErrorsSpiMain;
	Label m_OutErrorsSpiExternal;
	Label m_OutErrorsMainCAN;
	Label m_OutErrorsExternalCAN;
	Label m_OutCartesianX;
	Label m_OutCartesianY;
	Label m_OutCartesianZ;
	Label m_OutCartesianThetaX;
	Label m_OutCartesianThetaY;
	Label m_OutCartesianThetaZ;
	Label m_OutAngle1;
	Label m_OutAngle2;
	Label m_OutAngle3;
	Label m_OutAngle4;
	Label m_OutAngle5;
	Label m_OutAngle6;
	
	
	
	public MainWindow () : base(Gtk.WindowType.Toplevel)
	{
		PreBuild();
		DataInitialization();
		Build ();
		
		//Initialize the thread that will get the data from JACO
        m_WorkThread = new Thread(new ThreadStart(DoWork));
        m_WorkThread.Priority = ThreadPriority.Lowest;
        m_WorkThread.IsBackground = false;
		
		
	    m_WorkThread.Start();
	    mre.Set();
		
	}

	private void DataInitialization()
	{
		try
		{
			jaco = new CJacoArm(Crypto.GetInstance().Encrypt("MyValisPassword"));
			m_OutConnectionStatus.ModifyFg(StateType.Normal, m_ColorConnectionOpened);
			m_OutConnectionStatus.Text = "Jaco is connected.";
		}
		catch(Exception ex)
		{
			m_OutConnectionStatus.Text = "Jaco is not connected.";
		}
	}
	
	public void PreBuild()
	{
		InitializeGui();

		m_MainPanel.Put(m_MainTitle, 210, 30);
		m_MainPanel.Put(m_OutConnectionStatus, 390, 80);
		
		m_MainPanel.Put(m_TxtTimeAbsolute, 30, 120);
		m_MainPanel.Put(m_TxtTimeStartUp, 30, 140);
		m_MainPanel.Put(m_TxtTimeStampSavings, 30, 160);
		m_MainPanel.Put(m_TxtSupplyVoltage, 30, 180);
		m_MainPanel.Put(m_TxtCurrentConsumed, 30, 200);
		m_MainPanel.Put(m_TxtPowerConsumed, 30, 220);
		m_MainPanel.Put(m_TxtAveragePower, 30, 240);
		m_MainPanel.Put(m_TxtAccelerationX, 30, 260);
		m_MainPanel.Put(m_TxtAccelerationY, 30, 280);
		m_MainPanel.Put(m_TxtAccelerationZ, 30, 300);
		m_MainPanel.Put(m_TxtCodeVersion, 30, 320);
		m_MainPanel.Put(m_TxtCodeRevision, 30, 340);
		m_MainPanel.Put(m_TxtControlOperator, 30, 360);
		m_MainPanel.Put(m_TxtControlMode, 30, 380);
		m_MainPanel.Put(m_TxtHandMode, 30, 400);
		m_MainPanel.Put(m_TxtConnectedJointQuantity, 30, 420);
		m_MainPanel.Put(m_TxtPositionType, 30, 440);
		m_MainPanel.Put(m_TxtErrorsSpiMain, 30, 460);
		m_MainPanel.Put(m_TxtErrorsSpiExternal, 30, 480);
		m_MainPanel.Put(m_TxtErrorsMainCAN, 30, 500);
		m_MainPanel.Put(m_TxtErrorsExternalCAN, 30, 520);
		m_MainPanel.Put(m_TxtCartesianX, 30, 560);
		m_MainPanel.Put(m_TxtCartesianY, 30, 580);
		m_MainPanel.Put(m_TxtCartesianZ, 30, 600);
		m_MainPanel.Put(m_TxtCartesianThetaX, 30, 620);
		m_MainPanel.Put(m_TxtCartesianThetaY, 30, 640);
		m_MainPanel.Put(m_TxtCartesianThetaZ, 30, 660);
		m_MainPanel.Put(m_TxtAngle1, 500, 560);
		m_MainPanel.Put(m_TxtAngle2, 500, 580);
		m_MainPanel.Put(m_TxtAngle3, 500, 600);
		m_MainPanel.Put(m_TxtAngle4, 500, 620);
		m_MainPanel.Put(m_TxtAngle5, 500, 640);
		m_MainPanel.Put(m_TxtAngle6, 500, 660);
		
		m_MainPanel.Put(m_OutTimeAbsolute, 280, 120);
		m_MainPanel.Put(m_OutTimeStartUp, 280, 140);
		m_MainPanel.Put(m_OutTimeStampSavings, 280, 160);
		m_MainPanel.Put(m_OutSupplyVoltage, 280, 180);
		m_MainPanel.Put(m_OutCurrentConsumed, 280, 200);
		m_MainPanel.Put(m_OutPowerConsumed, 280, 220);
		m_MainPanel.Put(m_OutAveragePower, 280, 240);
		m_MainPanel.Put(m_OutAccelerationX, 280, 260);
		m_MainPanel.Put(m_OutAccelerationY, 280, 280);
		m_MainPanel.Put(m_OutAccelerationZ, 280, 300);
		m_MainPanel.Put(m_OutCodeVersion, 280, 320);
		m_MainPanel.Put(m_OutCodeRevision, 280, 340);
		m_MainPanel.Put(m_OutControlOperator, 280, 360);
		m_MainPanel.Put(m_OutControlMode, 280, 380);
		m_MainPanel.Put(m_OutHandMode, 280, 400);
		m_MainPanel.Put(m_OutConnectedJointQuantity, 280, 420);
		m_MainPanel.Put(m_OutPositionType, 280, 440);
		m_MainPanel.Put(m_OutErrorsSpiMain, 280, 460);
		m_MainPanel.Put(m_OutErrorsSpiExternal, 280, 480);
		m_MainPanel.Put(m_OutErrorsMainCAN, 280, 500);
		m_MainPanel.Put(m_OutErrorsExternalCAN, 280, 520);
		m_MainPanel.Put(m_OutCartesianX, 280, 560);
		m_MainPanel.Put(m_OutCartesianY, 280, 580);
		m_MainPanel.Put(m_OutCartesianZ, 280, 600);
		m_MainPanel.Put(m_OutCartesianThetaX, 280, 620);
		m_MainPanel.Put(m_OutCartesianThetaY, 280, 640);
		m_MainPanel.Put(m_OutCartesianThetaZ, 280, 660);
		m_MainPanel.Put(m_OutAngle1, 750, 560);
		m_MainPanel.Put(m_OutAngle2, 750, 580);
		m_MainPanel.Put(m_OutAngle3, 750, 600);
		m_MainPanel.Put(m_OutAngle4, 750, 620);
		m_MainPanel.Put(m_OutAngle5, 750, 640);
		m_MainPanel.Put(m_OutAngle6, 750, 660);
		
		Add(m_MainPanel);
	}
	
	public void InitializeGui()
	{
		m_ColorConnectionClosed = new Gdk.Color(255,0,0);
		m_ColorConnectionOpened = new Gdk.Color(0,255,0);
		Pango.FontDescription fontMaintTitle = Pango.FontDescription.FromString("Garamond 30");
		
		
		m_MainPanel = new Fixed();
		
		m_OutConnectionStatus = new Label("Jaco is not connected.");
		m_OutConnectionStatus.ModifyFg(StateType.Normal, m_ColorConnectionClosed);
		
		m_MainTitle = new Label("Kinova software example");
		
		m_TxtTimeAbsolute = new Label("Time absolute :");
		m_TxtTimeStartUp = new Label("Time from startup :");
		m_TxtTimeStampSavings = new Label("timestamp savings :");
		m_TxtSupplyVoltage = new Label("Supply voltage :");
		m_TxtCurrentConsumed = new Label("Current consumed :");
		m_TxtPowerConsumed = new Label("Power consumed :");
		m_TxtAveragePower = new Label("Average power :");
		m_TxtAccelerationX = new Label("Acceleration X :");
		m_TxtAccelerationY = new Label("Acceleration Y :");
		m_TxtAccelerationZ = new Label("Acceleration Z :");
		m_TxtCodeVersion = new Label("Code version :");
		m_TxtCodeRevision = new Label("Code revision :");
		m_TxtControlOperator = new Label("Control operator :");
		m_TxtControlMode = new Label("Control mode :");
		m_TxtHandMode = new Label("Hand mode :");
		m_TxtConnectedJointQuantity = new Label("Connected joint quantity :");
		m_TxtPositionType = new Label("Position type :");
		m_TxtErrorsSpiMain = new Label("Main SPI errors :");
		m_TxtErrorsSpiExternal = new Label("External SPI errors :");
		m_TxtErrorsMainCAN = new Label("Main CAN errors :");
		m_TxtErrorsExternalCAN = new Label("External CAN errors :");
		m_TxtCartesianX = new Label("Cartesian X :");
		m_TxtCartesianY = new Label("Cartesian Y :");
		m_TxtCartesianZ = new Label("Cartesian Z :");
		m_TxtCartesianThetaX = new Label("Cartesian Theta X :");
		m_TxtCartesianThetaY = new Label("Cartesian Theta Y :");
		m_TxtCartesianThetaZ = new Label("Cartesian Theta Z :");
		m_TxtAngle1 = new Label("Angle 1 :");
		m_TxtAngle2 = new Label("Angle 2 :");
		m_TxtAngle3 = new Label("Angle 3 :");
		m_TxtAngle4 = new Label("Angle 4 :");
		m_TxtAngle5 = new Label("Angle 5 :");
		m_TxtAngle6 = new Label("Angle 6 :");
		
		m_OutTimeAbsolute = new Label("????");
		m_OutTimeStartUp = new Label("????");
		m_OutTimeStampSavings = new Label("????");
		m_OutSupplyVoltage = new Label("????");
		m_OutCurrentConsumed = new Label("????");
		m_OutPowerConsumed = new Label("????");
		m_OutAveragePower = new Label("????");
		m_OutAccelerationX = new Label("????");
		m_OutAccelerationY = new Label("????");
		m_OutAccelerationZ = new Label("????");
		m_OutCodeVersion = new Label("????");
		m_OutCodeRevision = new Label("????");
		m_OutControlOperator = new Label("????");
		m_OutControlMode = new Label("????");
		m_OutHandMode = new Label("????");
		m_OutConnectedJointQuantity = new Label("????");
		m_OutPositionType = new Label("????");
		m_OutErrorsSpiMain = new Label("????");
		m_OutErrorsSpiExternal = new Label("????");
		m_OutErrorsMainCAN = new Label("????");
		m_OutErrorsExternalCAN = new Label("????");
		m_OutCartesianX = new Label("????");
		m_OutCartesianY = new Label("????");
		m_OutCartesianZ = new Label("????");
		m_OutCartesianThetaX = new Label("????");
		m_OutCartesianThetaY = new Label("????");
		m_OutCartesianThetaZ = new Label("????");
		m_OutAngle1 = new Label("????");
		m_OutAngle2 = new Label("????");
		m_OutAngle3 = new Label("????");
		m_OutAngle4 = new Label("????");
		m_OutAngle5 = new Label("????");
		m_OutAngle6 = new Label("????");
		
		m_MainTitle.ModifyFont(fontMaintTitle);
		
	}
	
	//Thread's work
    [STAThread]
    private void DoWork()
    {
		
		while (m_Run)
        {
				
			Gtk.Application.Invoke (delegate { 
				
				try
				{
					position = jaco.DiagnosticManager.DataManager.GetPositionLogLiveFromJaco();
				}
				catch(Exception)
				{
					System.Console.WriteLine("EXCEPTION");
				}
			});
			
			m_OutTimeAbsolute.Text = position.TimeAbsolute.ToString();
			m_OutTimeStartUp.Text = position.TimeFromStartup.ToString();
			m_OutTimeStampSavings.Text = position.TimeStampSavings.ToString();
			m_OutSupplyVoltage.Text = position.SupplyVoltage.ToString();
			m_OutCurrentConsumed.Text = position.CurrentConsumed.ToString();
			m_OutPowerConsumed.Text = position.PowerConsumed.ToString();
			m_OutAveragePower.Text = position.AveragePower.ToString();
			m_OutAccelerationX.Text = position.AccelerationCaptorX.ToString();
			m_OutAccelerationY.Text = position.AccelerationCaptorY.ToString();
			m_OutAccelerationZ.Text = position.AccelerationCaptorZ.ToString();
			m_OutCodeVersion.Text = position.CodeVersion.ToString("X1");
			m_OutCodeRevision.Text = position.CodeRevision.ToString("X1");
			m_OutControlOperator.Text = position.ControlOperator.ToString();
			m_OutControlMode.Text = position.ControlMode.ToString();
			m_OutHandMode.Text = position.HandMode.ToString();
			m_OutConnectedJointQuantity.Text = position.ConnectedJointQuantity.ToString();
			m_OutPositionType.Text = position.PositionType.ToString();
			m_OutErrorsSpiMain.Text = position.NbErrorsSpiPrincipal.ToString();
			m_OutErrorsSpiExternal.Text = position.NbErrorsSpiExternal.ToString();
			m_OutErrorsMainCAN.Text = position.NbErrorsCanPrincipal.ToString();
			m_OutErrorsExternalCAN.Text = position.NbErrorsCanExternal.ToString();
			m_OutCartesianX.Text = position.UserPosition.Position.Position[0].ToString();
			m_OutCartesianY.Text = position.UserPosition.Position.Position[1].ToString();
			m_OutCartesianZ.Text = position.UserPosition.Position.Position[2].ToString();
			m_OutCartesianThetaX.Text = position.UserPosition.Position.Rotation[0].ToString();
			m_OutCartesianThetaY.Text = position.UserPosition.Position.Rotation[1].ToString();
			m_OutCartesianThetaZ.Text = position.UserPosition.Position.Rotation[2].ToString();
			m_OutAngle1.Text = position.UserPosition.AnglesJoints.Angle[0].ToString();
			m_OutAngle2.Text = position.UserPosition.AnglesJoints.Angle[1].ToString();
			m_OutAngle3.Text = position.UserPosition.AnglesJoints.Angle[2].ToString();
			m_OutAngle4.Text = position.UserPosition.AnglesJoints.Angle[3].ToString();
			m_OutAngle5.Text = position.UserPosition.AnglesJoints.Angle[4].ToString();
			m_OutAngle6.Text = position.UserPosition.AnglesJoints.Angle[5].ToString();
			
			Thread.Sleep(100);
	    }
    }
	
	protected void OnDeleteEvent (object sender, DeleteEventArgs a)
	{
		m_Run = false;
		
		jaco.CloseAll();
		
		Application.Quit ();
		a.RetVal = true;
	}
}

