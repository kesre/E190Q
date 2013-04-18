using System;
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
        public double x_des, y_des, t_des;
        public double desiredX, desiredY, desiredT;
        private double desiredV, desiredW;
        public double pho, alpha, beta;
        public double deltaX, deltaY, deltaTh;

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
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 1.1;//8
        private double Kbeta = -1;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 60; //36; inside
        public short K_I = 0; //1;//0;
        public short K_D = 0;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p = 36, K_i = 1, K_d = 0, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public List<Particle> propagatedParticles;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 5000; //mm
        public double laserMinRange = 200;  //mm
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;
        private int laserTimer = 50000;
        private bool addRandomParticles = false;

        public class Particle
        {
            public double x, y, t, w;
            public bool isRand;

            public Particle()
            {
            }
        }

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        //// Motion Planner Variables 
        //public double samplingCellSizeX, samplingCellSizeY;
        //public int numOccupiedCells;
        //public int[] occupiedCellsList;
        //public int[] numNodesInCell;
        //public Node[,] NodesInCells;
        //public Node[] trajList, nodeList;
        
        //public int trajSize, trajCurrentNode, numNodes;

        public double originLat = 3406.38127;     //3406.364062;
        public double originLong = 11742.71995;    //11742.70931;
        public GPStoXY gpsToXY_est;
        public GPStoXY gpsToXY_des;
        double[] latList = { 3406.380105, 3406.364333, 3406.37696, 3406.374458, 3406.365414, 3406.362289, 3406.36103, 3406.365268, 3406.365044, 3406.363054 };
        double[] longList = { 11742.71921, 11742.68456, 11742.7183, 11742.71629, 11742.7182, 11742.72201, 11742.71994, 11742.71208, 11742.71031, 11742.71111 };
        public int gpsIndex = 0;
        public double distThreshold = 1;

        //public class Node
        //{
        //    public double x, y, t;
        //    public int lastNode;
        //    public int nodeIndex;

        //    public Node()
        //    {
        //        x = 0;
        //        y = 0;
        //        t = 0;
        //        lastNode = 0;
        //        nodeIndex = 0;
        //    }

        //    public Node(double _x, double _y, double _t, int _nodeIndex, int _lastNode)
        //    {
        //        x = _x;
        //        y = _y;
        //        t = _t;
        //        nodeIndex = _nodeIndex;
        //        lastNode = _lastNode;
        //    }
        //}

        public class GPStoXY
        {
            public double x, y, lat0, long0, x0, y0, lat1, long1, x1, y1;
            public double r = 6371451.9; //calculated radius of earth at 34.106333 N

            public GPStoXY()
            {
                x = 0;
                y = 0;
                lat0 = 0;
                long0 = 0;
            }

            public GPStoXY(double _x, double _y, double _lat0, double _long0)
            {
                x = _x;
                y = _y;        

                lat0 = ConvertDDDMM_MMMToDDD_DDD(_lat0);
                long0 = ConvertDDDMM_MMMToDDD_DDD(_long0);
                x0 = 0;
                y0 = 0;

                lat1 = ConvertDDDMM_MMMToDDD_DDD(3406.365843);
                long1 = ConvertDDDMM_MMMToDDD_DDD(11742.71287);
                //x1 = 30 * Math.Sin(25 * Math.PI / 180);
                //y1 = 30 * Math.Cos(25 * Math.PI / 180);    
                x1 = 2 * r * Math.Asin(Math.Sqrt(Math.Pow(Math.Cos(lat0), 2) * Math.Pow(Math.Sin((long1 - long0) / 2), 2)));
                y1 = 2 * r * Math.Asin(Math.Sin((lat1 - lat0) / 2));
                
            }

            public double ConvertDDDMM_MMMToDDD_DDD(double gpsCoord)
            {
                double newCoord = 0;
                double degrees, minutes;

                minutes = gpsCoord % 100;
                degrees = (gpsCoord - minutes) / 100;

                newCoord = (degrees + (minutes / 60)) * Math.PI / 180;
                //newCoord = gpsCoord * Math.PI / 180;
                return newCoord;
            }

            public void CalcCurXY(double currentLat, double currentLong)
            {
                // All instances of lat and long need to be divided by 100 due to formatting.

                currentLat = ConvertDDDMM_MMMToDDD_DDD(currentLat);
                currentLong = ConvertDDDMM_MMMToDDD_DDD(currentLong);

                //x = 2 * r * Math.Asin(Math.Sqrt(Math.Pow(Math.Cos(lat0), 2) * Math.Pow(Math.Sin((currentLong - long0) / 2), 2)));
                //y = 2 * r * Math.Asin(Math.Sin((currentLat - lat0) / 2));
                
                /*
                

                x = (currentLat - lat0) * (x1 - x0) / (lat1 - lat0);
                y = (currentLong - long0) * (y1 - y0) / (long1 - long0);
                */
                
                double distance = 2 * r * Math.Asin(Math.Sqrt(Math.Pow(Math.Sin((currentLat - lat0) / (2)), 2) +
                    Math.Cos(currentLat) * Math.Cos(lat0) * Math.Pow(Math.Sin((currentLong - long0) / (2)), 2)));
                double currentAngle = Math.Atan2((currentLat - lat0), (currentLong - long0));
                x = Math.Cos(currentAngle) * distance;
                y = Math.Sin(currentAngle) * distance;
                
            }
        }


        //Competition GPS & Way/checkpoint variables
        public int startPoint = 0;

        #endregion


        #region Navigation Setup

        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();

            // Create particles
            //particles = new Particle[numParticles];
            //propagatedParticles = new List<Particle>(numParticles);
            //for (int i = 0; i < numParticles; i++)
            //{
            //    particles[i] = new Particle();
            //    propagatedParticles.Add(new Particle());
            //}

            int numGPSAvg = 100;
            double totalLat = 0;
            double totalLong = 0;

            for (int i = 0; i < numGPSAvg; i++)
            {
                totalLat += jaguarControl.gpsRecord.latitude;
                totalLong += jaguarControl.gpsRecord.longitude;
            }

            //gpsToXY_est = new GPStoXY(0, 0, totalLat/numGPSAvg, totalLong/numGPSAvg);
            //gpsToXY_des = new GPStoXY(0, 0, totalLat / numGPSAvg, totalLong / numGPSAvg);
            gpsToXY_est = new GPStoXY(0, 0, 3406.379125, 11742.71771);
            gpsToXY_des = new GPStoXY(0, 0, 3406.379125, 11742.71771);

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
            x_des = 0;
            y_des = 0;
            t_des = 0;  //Maybe change later

            //desiredX = 0;// initialX;
            //desiredY = 0;// initialY;
            //desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            //occupiedCellsList = new int[numXCells * numYCells];
            //numNodesInCell = new int[numXCells * numYCells];
            //NodesInCells = new Node[numXCells * numYCells, 500];
            //trajList = new Node[maxNumNodes];
            //nodeList = new Node[maxNumNodes];
            //numNodes = 0;
            //trajList[0] = new Node(0, 0, 0, 0);
            //trajSize = 0;


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
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
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).

                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithOdometry();

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                //LocalizeEstWithParticleFilter();
 

                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    //if (motionPlanRequired)
                    //{
                        // Construct a new trajectory (lab 5)
                        // PRMMotionPlanner();
                    //    motionPlanRequired = false;
                    //}

                    // Drive the robot to a desired Point (lab 3)
                    //FlyToSetPoint();


                    LocalizeRealWithGPS();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

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
            accCalib_y = accCalib_y / numMeasurements;


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

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= laserTimer)
                {
                    for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t - 1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recent encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();

                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                    laserCounter = laserCounter + deltaT;
                    if (laserCounter >= laserTimer)
                    {
                        laserCounter = 0;
                        newLaserData = true;
                    }

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

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

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
                u_L = Math.Sign(u_L) * maxSignal;
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
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
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
                string qiText;
                if (jaguarControl.gpsRecord.qi == 0)
                {
                    qiText = "Fix not available";
                }
                else if (jaguarControl.gpsRecord.qi == 1)
                {
                    qiText = "Non DGPS fix available";
                }
                else if (jaguarControl.gpsRecord.qi == 2)
                {
                    qiText = "DGPS fix available";
                }
                else if (jaguarControl.gpsRecord.qi == 6)
                {
                    qiText = "Estimate";
                }
                else
                {
                    qiText = "Fix not available";
                }


                String newData = time.ToString() + "   " + jaguarControl.gpsRecord.latitude.ToString() + "   " + jaguarControl.gpsRecord.longitude.ToString() + "    " + qiText + "    " +
                        jaguarControl.imuRecord.magn_x.ToString() + "    " + jaguarControl.imuRecord.magn_y.ToString() + "    " + jaguarControl.imuRecord.magn_z.ToString() + "   " +
                        x_est.ToString() + "   " + y_est.ToString() + "   " + gpsIndex.ToString() + "   " + x_des.ToString() + "   " + y_des.ToString() + "   " + t_des.ToString();

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

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
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
            gpsToXY_des.CalcCurXY(latList[gpsIndex], longList[gpsIndex]);
            double distToCurrentNode = Math.Sqrt(Math.Pow(x_est - gpsToXY_des.x, 2) + Math.Pow(y_est - gpsToXY_des.y, 2));
            if (distToCurrentNode < distThreshold && gpsIndex + 1 < latList.Length)
            {
                gpsIndex++;
                gpsToXY_des.CalcCurXY(latList[gpsIndex], longList[gpsIndex]);
                x_des = gpsToXY_des.x;
                y_des = gpsToXY_des.y;
                t_des = Math.Atan2(y_des - y_est, x_des - x_est);
            }

            FlyToSetPoint();
        }

        #region Don't Care
        // This function houses the core motion planner. This function
        // will generate a new trajectory when called. Students must 
        // add their code here.

        //private void PRMMotionPlanner()
        //{
        //    // Initialize sampling grid cell variables for weighted
        //    // random selection of nodes to expand.
        //    samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
        //    samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
        //    numOccupiedCells = 0;
        //    for (int i = 0; i < numXCells * numYCells; i++)
        //        numNodesInCell[i] = 0;
        //    numNodes = 0;


        //    // ****************** Additional Student Code: Start ************

        //    // Put code here to expand the PRM until the goal node is reached,
        //    // or until a max number of iterations is reached.


        //    // Create and add the start Node
        //    Node startNode = new Node(x_est, y_est, 0, 0);
        //    AddNode(startNode);

        //    // Create the goal node
        //    Node goalNode = new Node(desiredX, desiredY, 0, 0);

        //    // Loop until path created
        //    bool pathFound = false;
        //    int maxIterations = maxNumNodes;
        //    int iterations = 0;
        //    Random randGenerator = new Random();
        //    int randCellNumber, randNodeNumber;
        //    Node randExpansionNode, newNode;
        //    double maxDist = 3.0, maxOrient = 6.28;
        //    double nextDist, nextOrient, newX, newY;

        //    if (!map.CollisionFound(startNode, goalNode, robotRadius))
        //    {
        //        goalNode.nodeIndex = 1;
        //        goalNode.lastNode = 0;
        //        AddNode(goalNode);
        //        pathFound = true;

        //        // Create the trajectory to follow
        //        BuildTraj(goalNode);
        //    }

        //    while (iterations < maxIterations && !pathFound)
        //    {
        //        randCellNumber = randGenerator.Next(numOccupiedCells);
        //        randNodeNumber = randGenerator.Next(numNodesInCell[occupiedCellsList[randCellNumber]]);
        //        randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];
        //        nextDist = randGenerator.NextDouble() * maxDist;
        //        nextOrient = (randGenerator.NextDouble() * maxOrient) - (maxOrient / 2);
        //        newX = randExpansionNode.x + nextDist * Math.Cos(nextOrient);
        //        newY = randExpansionNode.y + nextDist * Math.Sin(nextOrient);
        //        newNode = new Node(newX, newY, numNodes, randExpansionNode.nodeIndex);

        //        if (!map.CollisionFound(randExpansionNode, newNode, robotRadius))
        //        {
        //            AddNode(newNode);
        //        }

        //        if (!map.CollisionFound(nodeList[numNodes - 1], goalNode, robotRadius))
        //        {
        //            goalNode.nodeIndex = numNodes;
        //            goalNode.lastNode = numNodes - 1;
        //            AddNode(goalNode);
        //            pathFound = true;

        //            // Create the trajectory to follow
        //            BuildTraj(goalNode);
        //        }

        //        // Increment number of iterations
        //        iterations++;
        //    }
        //    // ****************** Additional Student Code: End   ************
        //}


        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        
        //int GetCellNumber(double x, double y)
        //{
        //    int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
        //    return cell;
        //}

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        //void AddNode(Node n)
        //{
        //    int cellNumber = GetCellNumber(n.x, n.y);
        //    if (numNodesInCell[cellNumber] < 1)
        //    {
        //        occupiedCellsList[numOccupiedCells] = cellNumber;
        //        numOccupiedCells++;
        //    }

        //    if (numNodesInCell[cellNumber] < 400)
        //    {
        //        NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
        //        numNodesInCell[cellNumber]++;

        //        // Add to nodelist
        //        nodeList[numNodes] = n;
        //        numNodes++;
        //    }
        //    return;
        //}


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        //void BuildTraj(Node goalNode)
        //{
        //    Node[] tempList = new Node[maxNumNodes];
        //    for (int j = 0; j < maxNumNodes; j++)
        //        trajList[j] = new Node(0, 0, 0, 0);

        //    tempList[0] = goalNode;
        //    int i = 1;

        //    // Make backwards traj by looking at parent of every child node
        //    while (tempList[i - 1].nodeIndex != 0)
        //    {
        //        tempList[i] = nodeList[tempList[i - 1].lastNode];
        //        i++;
        //    }

        //    // Reverse trajectory order
        //    for (int j = 0; j < i; j++)
        //    {
        //        trajList[j] = tempList[i - j - 1];
        //    }

        //    // Set size of trajectory and initialize node counter
        //    trajSize = i;
        //    trajCurrentNode = 0;

        //    return;
        //}




        #endregion
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

            deltaX = x_des - x_est;
            deltaY = y_des - y_est;
            deltaTh = NormalizeAngle(t_des - t_est);

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

        public void LocalizeRealWithGPS()
        {
            string qiText;
            jaguarControl.gpsRecord.longitude = 11742.71947;
            jaguarControl.gpsRecord.latitude = 3406.373997;
            if (jaguarControl.gpsRecord.qi == 0)
            {
                qiText = "Fix not available";
            }
            else if (jaguarControl.gpsRecord.qi == 1)
            {
                qiText = "Non DGPS fix available";
            }
            else if (jaguarControl.gpsRecord.qi == 2)
            {
                qiText = "DGPS fix available";
            }
            else if (jaguarControl.gpsRecord.qi == 6)
            {
                qiText = "Estimate";
            }
            else
            {
                qiText = "Fix not available";
            }

            gpsToXY_est.CalcCurXY(jaguarControl.gpsRecord.latitude, jaguarControl.gpsRecord.longitude);

            x_est = gpsToXY_est.x;
            y_est = gpsToXY_est.y;
            t_est = NormalizeAngle(t_est + angleTravelled);

            deltaX = x_des - x_est;
            deltaY = y_des - y_est;
            deltaTh = NormalizeAngle(t_des - t_est);

            pho = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
            alpha = NormalizeAngle(-t_est + Math.Atan2(deltaY, deltaX));

            if (Math.Abs(alpha) > Math.PI / 2)
            {
                alpha = NormalizeAngle(-t_est + Math.Atan2(-deltaY, -deltaX));
                pho *= -1;
            }

            beta = NormalizeAngle(deltaTh - alpha);
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }

        #region particles

        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab


            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF

            // x_est = 0; y_est = 0; t_est = 0;
            double leftWheelRand, rightWheelRand, randDistTravelled, randAngleTravelled;
            double u1, u2, u3, u4, u5, u6, norm1, norm2, norm3;

            for (int p = 0; p < numParticles; p++)
            {
                u1 = random.NextDouble(); //these are uniform(0,1) random doubles
                u2 = random.NextDouble();
                u3 = random.NextDouble();
                u4 = random.NextDouble();
                u5 = random.NextDouble();
                u6 = random.NextDouble();
                norm1 = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
                norm2 = Math.Sqrt(-2.0 * Math.Log(u3)) * Math.Sin(2.0 * Math.PI * u4); //random normal(0,1)
                norm3 = Math.Sqrt(-2.0 * Math.Log(u5)) * Math.Sin(2.0 * Math.PI * u6); //random normal(0,1)

                leftWheelRand = wheelDistanceL + norm1 * wheelDistanceL * 0.5;
                rightWheelRand = wheelDistanceR + norm2 * wheelDistanceR * 0.5;
                randDistTravelled = (leftWheelRand + rightWheelRand) / 2;
                randAngleTravelled = (rightWheelRand - leftWheelRand) / (2 * wheelRadius);

                propagatedParticles[p].x = particles[p].x + randDistTravelled * Math.Cos(particles[p].t + 0.5 * randAngleTravelled);
                propagatedParticles[p].y = particles[p].y + randDistTravelled * Math.Sin(particles[p].t + 0.5 * randAngleTravelled);
                //propagatedParticles[p].t = NormalizeAngle(particles[p].t + randAngleTravelled);
                propagatedParticles[p].t = particles[p].t + angleTravelled + norm3 * angleTravelled;
                propagatedParticles[p].isRand = particles[p].isRand;
                if (newLaserData && ((Math.Abs(wheelDistanceL) > 0) || (Math.Abs(wheelDistanceR) > 0)))
                {
                    // We will evaluate newLaserData later in this function.
                    propagatedParticles[p].w = CalculateWeight(propagatedParticles[p]);
                }
            }


            // TODO: Update Particle Pool (sort propogated Particles, cull based on entropy accumulated (maybe?))
            //propagatedParticles.Sort(
            //    delegate(Particle p1, Particle p2)
            //        {
            //            return p1.w.CompareTo(p2.w);
            //        }
            //);

            double propMax = 0;
            for (int i = 0; i < numParticles; i++)
            {
                propMax = Math.Max(propMax, propagatedParticles[i].w);
            }

            if (newLaserData && ((Math.Abs(wheelDistanceL) > 0) || (Math.Abs(wheelDistanceR) > 0)))
            {
                newLaserData = false;
                List<Particle> temp = new List<Particle>(numParticles);
                int maxOccur = 5;
                int occur = 0;
                int templen = 0;

                // Populate selection pool based on 'fitness'
                for (int i = 0; i < numParticles; i++)
                {
                    occur = (int)((propagatedParticles[i].w * maxOccur) / propMax);

                    for (int j = 0; j < occur; j++)
                    {
                        // Since occur isn't 0, there is a weight argument that the particle
                        // isn't completely random. Thus we'll use it to calculate our estimated
                        // position.
                        propagatedParticles[i].isRand = false;

                        // Add it for selection
                        temp.Add(propagatedParticles[i]);
                        templen++;
                    }
                }

                //if (addRandomParticles && templen < (maxOccur/2) * numParticles )
                //{
                // Add in new random particles.
                Particle p;
                for (int i = 0; i < numParticles / 5; i++)
                {
                    p = new Particle();
                    SetRandomPos(p);
                    temp.Add(p);
                    templen++;
                }
                //}

                if (templen > 0)
                {
                    // Choose new Population
                    for (int i = 0; i < numParticles; i++)
                    {
                        particles[i] = temp[random.Next(0, templen)];
                    }
                }
            }
            else
            {
                particles = propagatedParticles.ToArray();
            }

            double xSum = 0, ySum = 0, tSum = 0, totalWeight = 0;

            for (int i = 0; i < numParticles; i++)
            {
                if (!particles[i].isRand)
                {
                    xSum += particles[i].x * particles[i].w;
                    ySum += particles[i].y * particles[i].w;
                    tSum += particles[i].t * particles[i].w;
                    totalWeight += particles[i].w;
                }

            }

            //x_est = xSum / totalWeight;
            //y_est = ySum / totalWeight;
            //t_est = tSum / totalWeight;

            //for PRM testing
            
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        double CalculateWeight(Particle p)
        {
            // ****************** Additional Student Code: Start ************
            double weight = 0;
            double currentDistance = 0;
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                if ((LaserData[i] < laserMaxRange) && (LaserData[i] > laserMinRange))
                {
                    currentDistance = (1000 * map.GetClosestWallDistance(p.x, p.y, p.t - 1.57 + laserAngles[i]));
                    //weight += Math.Max((5000 - Math.Abs(LaserData[i] - currentDistance)), 0) / 
                    //  ((LaserData.Length / laserStepSize) * 6000);
                    weight += Math.Exp(-Math.Pow((LaserData[i] - currentDistance) / 100, 2));
                }
            }
            //weight = Math.Exp(10 * weight);
            return weight;
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles()
        {


            // Set particles in random locations and orientations within environment
            for (int i = 0; i < numParticles; i++)
            {

                // Either set the particles at known start position [0 0 0],  
                // or set particles at random locations.

                if (jaguarControl.startMode == 1)
                {
                    //SetRandomPos(particles[i]);
                }
                else
                {
                    //SetStartPos(particles[i]);
                }
            }

        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(Particle p)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
            // It might be helpful to use boundaries defined in the
            // Map.cs file (e.g. map.minX)
            double xRange = map.maxX - map.minX;
            double yRange = map.maxY - map.minY;
            double tRange = 2 * Math.PI;
            p.x = random.NextDouble() * xRange + map.minX;
            p.y = random.NextDouble() * yRange + map.minY;
            p.t = random.NextDouble() * tRange - Math.PI;
            p.isRand = true;
            p.w = random.NextDouble();


            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(Particle p)
        {
            p.x = initialX;
            p.y = initialY;
            p.t = initialT;
            p.w = 1;
        }


#endregion
        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
            double U1, U2, V1 = 0, V2;
            double S = 2.0;
            while (S >= 1.0)
            {
                U1 = random.NextDouble();
                U2 = random.NextDouble();
                V1 = 2.0 * U1 - 1.0;
                V2 = 2.0 * U2 - 1.0;
                S = Math.Pow(V1, 2) + Math.Pow(V2, 2);
            }
            double gauss = V1 * Math.Sqrt((-2.0 * Math.Log(S)) / S);
            return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
            if (a > 0)
                return 1.0;
            else if (a < 0)
                return -1.0;
            else
                return 0.0;
        }

        #endregion

    }
}
