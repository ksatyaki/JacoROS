using System;
using Gtk;
using Kinova.DLL.SafeGate;
using Kinova.API.Jaco;
using System.Threading;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.Data.Jaco.Diagnostic;


public partial class MainWindow : Gtk.Window
{
	//Main entrance to jaco's functionalities
	private CJacoArm jaco;
	
	Fixed m_MainPanel;
	Label m_OutConnectionStatus;
	
	Fixed m_GeneralPanel;
	Label m_TxtSerial;
	Label m_TxtModel;
	Label m_TxtSoftwareVersion;
	Label m_TxtClientID;
	Label m_TxtClientName;
	Label m_TxtClientOrganization;
	Label m_OutSerial;
	Label m_OutModel;
	Label m_OutSoftwareVersion;
	Entry m_InClientID;
	Entry m_InClientName;
	Entry m_InClientOrganization;
	Button m_BTNUpdateGeneral;
	Button m_BTNReadGeneral;
	
	Fixed m_ConfigurationPanel;
	Label m_TxtSensitivity;
	Label m_TxtMaxSpeed;
	Label m_TxtDrinkingHeight;
	Label m_TxtDrinkingDistance;
	Label m_TxtLaterality;
	Label m_TxtFingersInverted;
	Label m_TxtRetractAngle;
	Entry m_InSensitivity;
	Entry m_InMaxSpeed;
	Entry m_InDrinkingHeight;
	Entry m_InDrinkingDistance;
	Button m_BTNUpdateConfig;
	Button m_BTNReadConfig;
	RadioButton m_RadioRightHanded;
	RadioButton m_RadioLeftHanded;
	CheckButton m_CheckFingersInverted;
	Entry m_InRetractAngle;
	
	Fixed m_DiagnosisPanel;
	Label m_TxtAxis1Version;
	Label m_TxtAxis2Version;
	Label m_TxtAxis3Version;
	Label m_TxtAxis4Version;
	Label m_TxtAxis5Version;
	Label m_TxtAxis6Version;
	Label m_TxtFinger1Version;
	Label m_TxtFinger2Version;
	Label m_TxtFinger3Version;
	Label m_TxtMainCANVersion;
	Label m_TxtExtCANVersion;
	Label m_OutAxis1Version;
	Label m_OutAxis2Version;
	Label m_OutAxis3Version;
	Label m_OutAxis4Version;
	Label m_OutAxis5Version;
	Label m_OutAxis6Version;
	Label m_OutFinger1Version;
	Label m_OutFinger2Version;
	Label m_OutFinger3Version;
	Label m_OutMainCANVersion;
	Label m_OutExtCANVersion;
	Label m_TxtInfo;
	FileChooserButton m_InFileReprogram;
	Button m_BTNReprogram;
	Button m_BTNReadDiagnosis;
	
	Label m_MainTitle;
	Notebook m_MainTab;
	
	Label m_TabGeneral;
	Label m_TabConfiguration;
	Label m_TabDiagnosis;
	
	private CClientConfigurations m_CurrentClientConfig;
	private int[] m_CodeVersion;
	
	public MainWindow () : base(Gtk.WindowType.Toplevel)
	{	
		PreBuild();
		DataInitialization();
		Build ();
	}

	private void DataInitialization()
	{
		try
		{
			jaco = new CJacoArm(Crypto.GetInstance().Encrypt("MyValidPassword"));
			m_OutConnectionStatus.Text = "Jaco is connected.";
		}
		catch(Exception)
		{
			m_OutConnectionStatus.Text = "Jaco is not connected or your password is invalid.";
		}
		
		m_CurrentClientConfig = new CClientConfigurations();
	}
	
	public void PreBuild()
	{
		InitializeGui();
		
		m_GeneralPanel.Put(m_TxtSerial, 30, 320);
		m_GeneralPanel.Put(m_TxtModel, 30, 370);
		m_GeneralPanel.Put(m_TxtSoftwareVersion, 30, 420);
		m_GeneralPanel.Put(m_TxtClientID, 30, 30);
		m_GeneralPanel.Put(m_TxtClientName, 30, 80);
		m_GeneralPanel.Put(m_TxtClientOrganization, 30, 130);
		m_GeneralPanel.Put(m_OutSerial, 230, 320);
		m_GeneralPanel.Put(m_OutModel, 230, 370);
		m_GeneralPanel.Put(m_OutSoftwareVersion, 230, 420);
		m_GeneralPanel.Put(m_InClientID, 230, 27);
		m_GeneralPanel.Put(m_InClientName, 230, 77);
		m_GeneralPanel.Put(m_InClientOrganization, 230, 127);
		m_GeneralPanel.Put(m_BTNUpdateGeneral, 30, 187);
		m_GeneralPanel.Put(m_BTNReadGeneral, 180, 187);
		
		m_ConfigurationPanel.Put(m_TxtSensitivity, 30, 30);
		m_ConfigurationPanel.Put(m_TxtMaxSpeed, 30, 80);
		m_ConfigurationPanel.Put(m_TxtDrinkingHeight, 30, 130);
		m_ConfigurationPanel.Put(m_TxtDrinkingDistance, 30, 180);
		m_ConfigurationPanel.Put(m_TxtLaterality, 30, 230);
		m_ConfigurationPanel.Put(m_TxtFingersInverted, 30, 280);
		m_ConfigurationPanel.Put(m_TxtRetractAngle, 30, 330);
		m_ConfigurationPanel.Put(m_InSensitivity, 230, 30);
		m_ConfigurationPanel.Put(m_InMaxSpeed, 230, 80);
		m_ConfigurationPanel.Put(m_InDrinkingHeight, 230, 130);
		m_ConfigurationPanel.Put(m_InDrinkingDistance, 230, 180);
		m_ConfigurationPanel.Put(m_RadioRightHanded, 230, 215);
		m_ConfigurationPanel.Put(m_RadioLeftHanded, 230, 245);
		m_ConfigurationPanel.Put(m_CheckFingersInverted, 230, 280);
		m_ConfigurationPanel.Put(m_InRetractAngle, 230, 330);
		m_ConfigurationPanel.Put(m_BTNUpdateConfig, 30, 380);
		m_ConfigurationPanel.Put(m_BTNReadConfig, 180, 380);
		
		m_DiagnosisPanel.Put(m_TxtAxis1Version, 30, 30);
		m_DiagnosisPanel.Put(m_TxtAxis2Version, 30, 50);
		m_DiagnosisPanel.Put(m_TxtAxis3Version, 30, 70);
		m_DiagnosisPanel.Put(m_TxtAxis4Version, 30, 90);
		m_DiagnosisPanel.Put(m_TxtAxis5Version, 30, 110);
		m_DiagnosisPanel.Put(m_TxtAxis6Version, 30, 130);
		m_DiagnosisPanel.Put(m_TxtFinger1Version, 30, 150);
		m_DiagnosisPanel.Put(m_TxtFinger2Version, 30, 170);
		m_DiagnosisPanel.Put(m_TxtFinger3Version, 30, 190);
		m_DiagnosisPanel.Put(m_TxtMainCANVersion, 30, 210);
		m_DiagnosisPanel.Put(m_TxtExtCANVersion, 30, 230);
		m_DiagnosisPanel.Put(m_OutAxis1Version, 230, 30);
		m_DiagnosisPanel.Put(m_OutAxis2Version, 230, 50);
		m_DiagnosisPanel.Put(m_OutAxis3Version, 230, 70);
		m_DiagnosisPanel.Put(m_OutAxis4Version, 230, 90);
		m_DiagnosisPanel.Put(m_OutAxis5Version, 230, 110);
		m_DiagnosisPanel.Put(m_OutAxis6Version, 230, 130);
		m_DiagnosisPanel.Put(m_OutFinger1Version, 230, 150);
		m_DiagnosisPanel.Put(m_OutFinger2Version, 230, 170);
		m_DiagnosisPanel.Put(m_OutFinger3Version, 230, 190);
		m_DiagnosisPanel.Put(m_OutMainCANVersion, 230, 210);
		m_DiagnosisPanel.Put(m_OutExtCANVersion, 230, 230);
		m_DiagnosisPanel.Put(m_InFileReprogram, 30, 330);
		m_DiagnosisPanel.Put(m_BTNReprogram, 30, 360);
		m_DiagnosisPanel.Put(m_BTNReadDiagnosis, 30, 280);
		m_DiagnosisPanel.Put(m_TxtInfo, 30, 390);
		
		m_MainPanel.Put(m_MainTab, 50, 100);
		m_MainPanel.Put(m_MainTitle, 50, 30);
		m_MainPanel.Put(m_OutConnectionStatus, 50, 620);
		
		Add(m_MainPanel);
	}
	
	public void InitializeGui()
	{
		m_MainPanel = new Fixed();
		m_OutConnectionStatus = new Label("Jaco is not connected.");
		
		m_GeneralPanel = new Fixed();
		m_TxtSerial = new Label("Jaco serial # :");
	    m_TxtModel = new Label("Jaco model :");
	    m_TxtSoftwareVersion = new Label("Software version :");
		m_TxtClientID = new Label("Client ID :");
	    m_TxtClientName = new Label("Client name :");
	    m_TxtClientOrganization = new Label("Client organization :");
		m_OutSerial = new Label("????");
	    m_OutModel = new Label("????");
	    m_OutSoftwareVersion = new Label("????");
		m_InClientID = new Entry("");
	    m_InClientName = new Entry("");
	    m_InClientOrganization = new Entry("");
		m_BTNUpdateGeneral = new Button("Update Jaco");
		m_BTNUpdateGeneral.Clicked += new EventHandler(BTN_UpdateClick);
		m_BTNReadGeneral = new Button("Read from Jaco");
		m_BTNReadGeneral.Clicked += new EventHandler (BTN_ReadClick);
		
		m_ConfigurationPanel = new Fixed();
		m_TxtSensitivity = new Label("Control's sensitivity :");
		m_TxtMaxSpeed = new Label("Permitted max speed :");
		m_TxtDrinkingHeight = new Label("Drinking height :");
		m_TxtDrinkingDistance = new Label("Drinking distance :");
		m_TxtLaterality = new Label("Laterality :");
		m_TxtFingersInverted = new Label("Fingers inverted :");
		m_TxtRetractAngle = new Label("Retracted position angle :");
		m_InSensitivity = new Entry("");
		m_InMaxSpeed = new Entry("");
		m_InDrinkingHeight = new Entry("");
		m_InDrinkingDistance = new Entry("");
		m_RadioRightHanded = new RadioButton("Righthanded");
		m_RadioLeftHanded = new RadioButton("Lefthanded");
		m_RadioLeftHanded.Group = m_RadioRightHanded.Group;
		m_CheckFingersInverted = new CheckButton("Invert fingers");
		m_InRetractAngle = new Entry("");
		m_BTNUpdateConfig = new Button("Update Jaco");
		m_BTNUpdateConfig.Clicked += new EventHandler(BTN_UpdateClick);
		m_BTNReadConfig = new Button("Read from Jaco");
		m_BTNReadConfig.Clicked += new EventHandler(BTN_ReadClick);
		
		m_DiagnosisPanel = new Fixed();
		m_TxtAxis1Version = new Label("Axis 1 version :");
		m_TxtAxis2Version = new Label("Axis 2 version :");
		m_TxtAxis3Version = new Label("Axis 3 version :");
		m_TxtAxis4Version = new Label("Axis 4 version :");
		m_TxtAxis5Version = new Label("Axis 5 version :");
		m_TxtAxis6Version = new Label("Axis 6 version :");
		m_TxtFinger1Version = new Label("Finger 1 version :");
		m_TxtFinger2Version = new Label("Finger 2 version :");
		m_TxtFinger3Version = new Label("Finger 3 version :");
		m_TxtMainCANVersion = new Label("Main CAN version :");
		m_TxtExtCANVersion = new Label("Ext CAN Version :");
		m_OutAxis1Version = new Label("????");
		m_OutAxis2Version = new Label("????");
		m_OutAxis3Version = new Label("????");
		m_OutAxis4Version = new Label("????");
		m_OutAxis5Version = new Label("????");
		m_OutAxis6Version = new Label("????");
		m_OutFinger1Version = new Label("????");
		m_OutFinger2Version = new Label("????");
		m_OutFinger3Version = new Label("????");
		m_OutMainCANVersion = new Label("????");
		m_OutExtCANVersion = new Label("????");
		m_TxtInfo = new Label("");
		m_InFileReprogram = new FileChooserButton("Select a file", FileChooserAction.Open);
		m_InFileReprogram.WidthRequest = 440;
		m_BTNReprogram = new Button("Reprogram Jaco");
		m_BTNReprogram.Clicked += new EventHandler(BTN_ReprogramClick);
		m_BTNReadDiagnosis = new Button("Read from Jaco");
		m_BTNReadDiagnosis.Clicked += new EventHandler(BTN_ReadClick);
		
		m_MainTitle = new Label("Kinova software example");
		Pango.FontDescription fontMaintTile = Pango.FontDescription.FromString("Garamond 30");
		m_MainTitle.ModifyFont(fontMaintTile);
		
		m_TabGeneral = new Label("General");
		m_TabConfiguration = new Label("Configuration");
		m_TabDiagnosis = new Label("Diagnosis");
		
		m_MainTab = new Notebook();
		m_MainTab.WidthRequest = 500;
		m_MainTab.HeightRequest = 500;
		m_MainTab.AppendPage(m_GeneralPanel, m_TabGeneral);
		m_MainTab.AppendPage(m_ConfigurationPanel, m_TabConfiguration);
		m_MainTab.AppendPage(m_DiagnosisPanel, m_TabDiagnosis);
	}
	
	private void UpdateGui()
	{
		m_OutSerial.Text = m_CurrentClientConfig.SerialNo;
		m_OutModel.Text = m_CurrentClientConfig.ModelNo;
		m_OutSoftwareVersion.Text = m_CodeVersion[0].ToString("X1");
		m_InClientID.Text = m_CurrentClientConfig.ClientNo;
		m_InClientName.Text = m_CurrentClientConfig.ClientName;
		m_InClientOrganization.Text = m_CurrentClientConfig.Organization;
		
		m_InSensitivity.Text = m_CurrentClientConfig.Sensibility.ToString();
		m_InMaxSpeed.Text = m_CurrentClientConfig.MaxLinearSpeed.ToString();
		m_InDrinkingHeight.Text = m_CurrentClientConfig.DrinkingHeight.ToString();
		m_InDrinkingDistance.Text = m_CurrentClientConfig.DrinkingDistance.ToString();
		
		if(m_CurrentClientConfig.Laterality == Kinova.DLL.Data.Jaco.CJacoStructures.ArmLaterality.RightHandedness)
		{
			m_RadioRightHanded.Active = true;
			m_RadioLeftHanded.Active = false;
		}
		else
		{
			m_RadioRightHanded.Active = false;
			m_RadioLeftHanded.Active = true;
		}
		
		if(m_CurrentClientConfig.Fingers2and3inverted == 0)
		{
			m_CheckFingersInverted.Active = false;
		}
		else
		{
			m_CheckFingersInverted.Active = true;
		}
		
		m_InRetractAngle.Text = m_CurrentClientConfig.AngleRetractedPosition.ToString();
		
		m_OutAxis1Version.Text = m_CodeVersion[1].ToString("X1");
		m_OutAxis2Version.Text = m_CodeVersion[2].ToString("X1");
		m_OutAxis3Version.Text = m_CodeVersion[3].ToString("X1");
		m_OutAxis4Version.Text = m_CodeVersion[4].ToString("X1");
		m_OutAxis5Version.Text = m_CodeVersion[5].ToString("X1");
		m_OutAxis6Version.Text = m_CodeVersion[6].ToString("X1");
		
		m_OutFinger1Version.Text = m_CodeVersion[7].ToString("X1");
		m_OutFinger2Version.Text = m_CodeVersion[8].ToString("X1");
		m_OutFinger3Version.Text = m_CodeVersion[9].ToString("X1");
		
		m_OutMainCANVersion.Text = m_CodeVersion[11].ToString("X1");
		m_OutExtCANVersion.Text = m_CodeVersion[12].ToString("X1");
	}
	
	private void BTN_ReadClick(object sender, System.EventArgs e) 
	{
		
		Gtk.Application.Invoke (delegate {
			try
			{
				m_CurrentClientConfig = jaco.ConfigurationsManager.GetClientConfigurations();
				m_CodeVersion = jaco.ConfigurationsManager.GetCodeVersion();

				UpdateGui();
			}
			catch(Exception)
			{
				
			}
		});
	}
	
	private void BTN_ReprogramClick(object sender, System.EventArgs e)
	{
		
		string path = m_InFileReprogram.Filename;
		
		if(path != (null))
		{
			if(!path.Equals(""))
			{
				m_TxtInfo.Text = "Reprogramming...";
				jaco.DiagnosticManager.ToolManager.ReprogrammingJacoArm(path);
				m_TxtInfo.Text = "Done";
			}
			else
			{
				m_TxtInfo.Text = "The file path must not be empty";
			}
		}
		else
		{
			m_TxtInfo.Text = "The file path must not be null";
		}
	}
	
	private void BTN_UpdateClick(object sender, System.EventArgs e)
	{
		Gtk.Application.Invoke (delegate {
			try
			{
				CClientConfigurations configTemp = jaco.ConfigurationsManager.GetClientConfigurations();
				
				configTemp.AngleRetractedPosition = Int32.Parse(m_InRetractAngle.Text);
				configTemp.ClientName = m_InClientName.Text;
				configTemp.ClientNo = m_InClientID.Text;
				configTemp.Organization = m_InClientOrganization.Text;
				configTemp.DrinkingDistance = float.Parse(m_InDrinkingDistance.Text);
				configTemp.DrinkingHeight = float.Parse(m_InDrinkingHeight.Text);
				if(m_CheckFingersInverted.Active)
				{
					configTemp.Fingers2and3inverted = 1;
				}
				else
				{
					configTemp.Fingers2and3inverted = 0;
				}
				configTemp.Sensibility = float.Parse(m_InSensitivity.Text);
				configTemp.MaxLinearSpeed = float.Parse(m_InMaxSpeed.Text);
				
				if(m_RadioRightHanded.Active)
				{
					configTemp.Laterality = Kinova.DLL.Data.Jaco.CJacoStructures.ArmLaterality.RightHandedness;
				}
				else
				{
					configTemp.Laterality = Kinova.DLL.Data.Jaco.CJacoStructures.ArmLaterality.LeftHandedness;
				}
				
				jaco.ConfigurationsManager.SetClientConfigurations(configTemp);
				
			}
			catch(Exception)
			{
				
			}
		});
	}
	
	protected void OnDeleteEvent (object sender, DeleteEventArgs a)
	{
		jaco.CloseAll();
		Application.Quit ();
		a.RetVal = true;
	}
}

