﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;
        public double deltaX, deltaY, deltaTh;
        private double desiredV, desiredW;
        public double pho, alpha, beta;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 50;
        private static int encoderMax = 32767;
        public static int pulsesPerRotation = 190;
        public static double wheelRadius = 0.089;
        public double robotRadius = 0.2305;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private static double maxVelocity = 0.25;
        private short maxPulses = (short)(maxVelocity * pulsesPerRotation / (2 * Math.PI * wheelRadius));
        public double Kpho = 1;//1;
        public double Kalpha = 1.1;//2;//8
        public double Kbeta = -1;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        // Straight
        //double K_p = 25;
        //double K_i = 15;
        //double K_d = 10;

        public double K_p = 25;// 
        public double K_i = 12;// 12 / deltaT;//20
        public double K_d = 13; //100.1;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        private static Genetic genAlg_;
        private int numGenerations = 5; 
        private int popSize = 100; 
        private int mutationRate = 10;
        private int mutationFactor = 10; 
        private int numParents = 20;
        private int maxSteps = 500;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            genAlg_ = new Genetic(numGenerations, popSize, mutationRate, mutationFactor, this, numParents, maxSteps);
            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            genAlg_ = new Genetic(numGenerations, popSize, mutationRate, mutationFactor, this, numParents, maxSteps);
            InternalReset();
        }

        public void InternalReset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 3, we just call the function 
                // FlyToSetPoint().
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)  
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            short maxPosOutput = 32767;
            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);
            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(-maxPosOutput, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(-maxPosOutput, (int)motorSignalR));

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // We will use the desiredRotRateRs to set our PWM signals
            double cur_e_R = desiredRotRateR - (diffEncoderPulseR * 1000 / (double)deltaT);// *1000 for conversion to seconds from milliseconds
            double cur_e_L = desiredRotRateL - (diffEncoderPulseL * 1000 / (double)deltaT);
            double e_dir_R = (cur_e_R - e_R) * 1000 / (double)deltaT;
            double e_dir_L = (cur_e_L - e_L) * 1000 / (double)deltaT;
            e_R = cur_e_R;
            e_L = cur_e_L;

            double maxErr = (300 / (double)deltaT);

            u_R = K_p * e_R + K_i * e_sum_R + K_d * e_dir_R;
            u_L = K_p * e_L + K_i * e_sum_L + K_d * e_dir_L;

            double minSignal = 3000;
            double maxSignal = 16000;
            double zeroSignal = 20;
            double org_u_R = u_R;
            double org_u_L = u_L;

            double LRRatio = Math.Abs(u_L / u_R);

            if (LRRatio > (maxSignal / minSignal))
            {
                u_L = Math.Sign(u_L) *maxSignal;
                u_R = Math.Sign(u_R) * minSignal;
            }
            else if (LRRatio < (minSignal / maxSignal))
            {
                u_R = Math.Sign(u_R) * maxSignal;
                u_L = Math.Sign(u_L) * minSignal;
            }
            else if ((Math.Abs(u_L) > maxSignal) || (Math.Abs(u_R) > maxSignal))
            {
                if (LRRatio > 1)
                {
                    u_L = Math.Sign(u_L) * maxSignal;
                    u_R = Math.Sign(u_R) * maxSignal / LRRatio;
                }
                else
                {
                    u_R = Math.Sign(u_R) * maxSignal;
                    u_L = Math.Sign(u_L) * maxSignal * LRRatio;
                }
            }
            else if ((Math.Abs(u_L) < minSignal) || (Math.Abs(u_R) < minSignal))
            {
                if (LRRatio > 1)
                {
                    u_R = Math.Sign(u_R) * minSignal;
                    u_L = Math.Sign(u_L) * minSignal * LRRatio;
                }
                else
                {
                    u_L = Math.Sign(u_L) * minSignal;
                    u_R = Math.Sign(u_R) * minSignal / LRRatio;
                }
            }

            if (Math.Abs(org_u_L) < zeroSignal)
            {
                u_L = 0;
            }
            if (Math.Abs(org_u_R) < zeroSignal)
            {
                u_R = 0;
            }

            motorSignalL = (short)(zeroOutput + u_L);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            e_sum_R = Math.Max(-maxErr, Math.Min(0.90 * e_sum_R + e_R * (deltaT / 1000), maxErr));
            e_sum_L = Math.Max(-maxErr, Math.Min(0.90 * e_sum_L + e_L * (deltaT / 1000), maxErr));

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Hour.ToString() + "-" + DateTime.Now.Minute.ToString() + "-" + DateTime.Now.Second.ToString();
            logFile = File.CreateText("JaguarData_" + "_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() + " | " +
                        pho.ToString() + " " + alpha.ToString() + " " + beta.ToString() + " | " + 
                        desiredV.ToString() + " " + desiredW.ToString() + " | " +
                        motorSignalL.ToString() + " " + motorSignalR.ToString() + " | " +
                        Kpho.ToString() + " " + Kalpha.ToString() + " " + Kbeta.ToString() + " | " +
                        K_p.ToString() + " " + K_i.ToString() + " " + K_d.ToString();

                    logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!


            // Uncomment this to use genetic Algorithm. (TODO: add a way to view the changing K values.)
            //double[] newKs = genAlg_.Step(pho, deltaTh);
            // //Kpho = newKs[0];   // Don't use this for now - Stability is time-expensive!
            // //Kalpha = newKs[1]; // Don't use this for now - Stability is time-expensive!
            // //Kbeta = newKs[2];  // Don't use this for now - Stability is time-expensive!
            //K_p = newKs[3];
            //K_i = newKs[4];
            //K_d = newKs[5];
            
            double omegaL, omegaR;
            double dPhiL, dPhiR;

            // pho will be negative if the robot should be moving backwards.
            desiredV = Kpho * pho;
            if (Math.Abs(pho) > .1)
            {  
                desiredW = Kalpha * alpha + Kbeta * beta;
            }
            else if (Math.Abs(deltaTh) > .2)
            {
                desiredW = .3 * deltaTh;
            }
            else
            {
                desiredV = 0;
                desiredW = 0;
            }
      
            omegaL = .5 * (desiredV - desiredW / robotRadius);
            omegaR = .5 * (desiredV + desiredW / robotRadius);

            dPhiL = omegaL * robotRadius * 2 / wheelRadius;
            dPhiR = omegaR * robotRadius * 2 / wheelRadius;

            desiredRotRateL = (short)(dPhiL * pulsesPerRotation / (2 * Math.PI));
            desiredRotRateR = (short)(dPhiR * pulsesPerRotation / (2 * Math.PI));
            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        public double NormalizeAngle(double angle)
        {
            if (Math.Abs(angle) > Math.PI)
            {
                angle -= Math.Sign(angle) * 2 * Math.PI;
            }
            return angle;
        }

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.
            double diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            double diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // Check for encoder pulse rollover
            if (diffEncoderPulseR > encoderMax / 2)
            {
                diffEncoderPulseR = diffEncoderPulseR - encoderMax;
            }

            else if (diffEncoderPulseR < -encoderMax / 2)
            {
                diffEncoderPulseR = diffEncoderPulseR + encoderMax;
            }

            if (diffEncoderPulseL > encoderMax / 2)
            {
                diffEncoderPulseL = diffEncoderPulseL - encoderMax;
            }

            else if (diffEncoderPulseL < -encoderMax / 2)
            {
                diffEncoderPulseL = diffEncoderPulseL + encoderMax;
            }

            diffEncoderPulseR = -diffEncoderPulseR;

            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;

            wheelDistanceR = diffEncoderPulseR / pulsesPerRotation * 2 * Math.PI * wheelRadius;
            wheelDistanceL = diffEncoderPulseL / pulsesPerRotation * 2 * Math.PI * wheelRadius;

            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);



            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x = x + distanceTravelled * Math.Cos(t + 0.5 * angleTravelled);
            y = y + distanceTravelled * Math.Sin(t + 0.5 * angleTravelled);
            t = t + angleTravelled;

            // Ensure theta value stays between Pi and -Pi
            t = NormalizeAngle(t);

            x_est = x;
            y_est = y;
            t_est = t;

            deltaX = desiredX - x_est;
            deltaY = desiredY - y_est;
            deltaTh = NormalizeAngle(desiredT - t_est);

            pho = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
            alpha = NormalizeAngle(-t_est + Math.Atan2(deltaY, deltaX));            

            if (Math.Abs(alpha) > Math.PI / 2)
            {
                alpha = NormalizeAngle(-t_est + Math.Atan2(-deltaY, -deltaX));
                pho *= -1;
            }

            beta = NormalizeAngle(deltaTh - alpha);



            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}
