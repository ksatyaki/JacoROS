using Kinova.API.Jaco.Configurations;
using System.IO;
using Kinova.API.Jaco.Diagnostic;
using System.Collections.Generic;
using System;
using System.Xml.Serialization;
using System.Xml;
using System.Threading;
using Kinova.API.Jaco.Control;
using Kinova.API.Jaco.ValidationTools.Exceptions;
using Kinova.DLL.Data.Jaco;
using Kinova.DLL.Data.Util;
using Kinova.DLL.Data.Jaco.Diagnostic;
using Kinova.DLL.Data.Path;
using Kinova.API.Jaco;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.SafeGate;


namespace KinovaExample_Position
{
    //Position data modification example
    public class ExampleProgram
    {
        //Your password provided by Kinova to access the API
        private const string m_APIpass = "MyPassword";

        //Your JACO
        private CJacoArm jaco;

        //Constructor
        public ExampleProgram() 
        {
            try
            {
                //Initialize JACO Object with an encrypted valid password
                jaco = new CJacoArm(Crypto.GetInstance().Encrypt(m_APIpass));
            }
            catch(CAccessDeniedException ex)
            {
                throw ex;
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void ExecuteExample()
        {
            //If JACO is connected
            if (jaco.JacoIsReady())
            {
                System.Console.WriteLine("JACO is up and ready...");
                System.Console.WriteLine("");
                System.Console.WriteLine("");

                //We load position data from JACO.
                CPosition JacoInformation = jaco.DiagnosticManager.DataManager.GetPositionLogLiveFromJaco();

                //We load client data from JACO
                CClientConfigurations ClientConfig = jaco.ConfigurationsManager.GetClientConfigurations();

                //Display the data
                System.Console.WriteLine("*** INFORMATION ABOUT CLIENT ***");
                System.Console.WriteLine("");
                System.Console.WriteLine("    Organization = " + ClientConfig.Organization);
                System.Console.WriteLine("     Client name = " + ClientConfig.ClientName);
                System.Console.WriteLine("       Client no = " + ClientConfig.ClientNo);
                System.Console.WriteLine("       Serial no = " + ClientConfig.SerialNo);
                System.Console.WriteLine(" JACO laterality = " + ClientConfig.Laterality);
                System.Console.WriteLine("");
                System.Console.WriteLine("");

                System.Console.WriteLine("*** INFORMATION ABOUT POSITION ***");
                System.Console.WriteLine("");
                System.Console.WriteLine("       DSP code version = " + JacoInformation.CodeVersion.ToString("x1"));
                System.Console.WriteLine("                Voltage = " + JacoInformation.SupplyVoltage + " V");
                System.Console.WriteLine("       Current consumed = " + JacoInformation.CurrentConsumed + " A");
                System.Console.WriteLine(" Average power consumed = " + JacoInformation.AveragePower + " W");
                System.Console.WriteLine("    Joint Connected QTY = " + JacoInformation.ConnectedJointQuantity + " joints");

                //if the joystick is connected
                if (JacoInformation.SystemStatus.JoystickActive == 1)
                {
                    System.Console.WriteLine("        Joystick Status = ACTIVE");
                }
                else
                {
                    System.Console.WriteLine("        Joystick Status = NOT ACTIVE");
                }

                System.Console.WriteLine("");
                System.Console.WriteLine("    Current on Joint #1 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[0] + " A");
                System.Console.WriteLine("    Current on Joint #2 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[1] + " A");
                System.Console.WriteLine("    Current on Joint #3 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[2] + " A");
                System.Console.WriteLine("    Current on Joint #4 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[3] + " A");
                System.Console.WriteLine("    Current on Joint #5 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[4] + " A");
                System.Console.WriteLine("    Current on Joint #6 = " + JacoInformation.UserCurrent.AnglesJoints.Angle[5] + " A");
                System.Console.WriteLine("");
                System.Console.WriteLine("      Force on Joint #1 = " + JacoInformation.UserForce.AnglesJoints.Angle[0] + " N");
                System.Console.WriteLine("      Force on Joint #2 = " + JacoInformation.UserForce.AnglesJoints.Angle[1] + " N");
                System.Console.WriteLine("      Force on Joint #3 = " + JacoInformation.UserForce.AnglesJoints.Angle[2] + " N");
                System.Console.WriteLine("      Force on Joint #4 = " + JacoInformation.UserForce.AnglesJoints.Angle[3] + " N");
                System.Console.WriteLine("      Force on Joint #5 = " + JacoInformation.UserForce.AnglesJoints.Angle[4] + " N");
                System.Console.WriteLine("      Force on Joint #6 = " + JacoInformation.UserForce.AnglesJoints.Angle[5] + " N");
                
            }
            //JACO is not connected
            else
            {
                System.Console.WriteLine("JACO is offline");
            }
        }

        [STAThread]
        static void Main(string[] args)
        {
            
            try
            {
                //An ExempleProgram instance
                ExampleProgram app = new ExampleProgram();

                //we launch the example
                app.ExecuteExample();
            }
            catch (CAccessDeniedException ex)
            {
                System.Console.WriteLine("ACCESS DENIED - Password provided is invalid");
            }
            catch (Exception ex)
            {
                System.Console.WriteLine("Cannot connect to jaco");
            }
            

            System.Console.WriteLine("End of the example...");
            System.Console.ReadKey();

        }
    }
}
