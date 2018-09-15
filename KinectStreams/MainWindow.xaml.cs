using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Firebase.Database;
using Firebase.Database.Query;
using System.Threading.Tasks;
using System.Threading;

namespace KinectStreams
{
    public struct SensorPosition
    {
        public double x, y;

        public SensorPosition(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members


        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        FirebaseClient _firebase;
        Dictionary<ulong, double[]> _body_positions;
        double[] _signal_strengths;
        SensorPosition ANTENNA_POSITION_0 = new SensorPosition(-30.0, 0.0);
        SensorPosition ANTENNA_POSITION_1 = new SensorPosition(30.0, 0.0);
        SensorPosition ANTENNA_POSITION_2 = new SensorPosition(0.0, -30.0);



        #endregion

        #region Constructor

        public MainWindow()
        {
            InitializeComponent();
        }

        #endregion

        #region Event handlers

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                _body_positions = new Dictionary<ulong, double[]>();
                _firebase = new FirebaseClient("https://htn2018-acba7.firebaseio.com");
                Thread firebase_thread = new Thread(firebasePollLoop);
                firebase_thread.Start();

            }
        }

        private void firebasePollLoop()
        {

        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            //var observable = firebase.Child("test").AsObservable<string>().Subscribe(s => Console.WriteLine(s));

            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    camera.Source = frame.ToBitmap();
                }
            }

            // Get most recent power readings from antennae.
            getSignalStrengths();

            // Get angle from signal strengths.
            double angle = getAngle(_signal_strengths);
            //double angle = getRobustAngle(_signal_strengths);
            Console.WriteLine("Angle = {0}", angle);

            // Body
            var tracking_ids = new List<ulong>();
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];

                    frame.GetAndRefreshBodyData(_bodies);

                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {
                            if (body.IsTracked)
                            {
                                // Keep track of all the skeletons tracked this iteration.
                                tracking_ids.Add(body.TrackingId);
                                var spine_mid = body.Joints[JointType.SpineMid];
                                CameraSpacePoint cameraPoint = spine_mid.Position;
                                double[] position = {cameraPoint.X, cameraPoint.Y, cameraPoint.Z}; 
                                _body_positions[body.TrackingId] = position;
                                List<ulong> stale_keys = new List<ulong>(); 

                                // Iterate through dictionary, and remove stale tracking id's.
                                foreach (KeyValuePair<ulong, double[]> entry in _body_positions)
                                {
                                    if (!tracking_ids.Contains(entry.Key)) {
                                        stale_keys.Add(entry.Key);
                                    }
                                    else
                                    {
                                        Console.WriteLine("Tracking ID = {0}", entry.Key);
                                        Console.WriteLine("Positions = ({0}, {1}, {2})", entry.Value[0], entry.Value[1], entry.Value[2]);
                                    }
                                }

                                foreach (ulong stale_key in stale_keys)
                                {
                                    _body_positions.Remove(stale_key);
                                } 

                              


                                // Draw skeleton.
                                canvas.DrawSkeleton(body);
                            }
                        }
                    }
                }
            }
        }

        // Get the signal strengths from FireBase.
        // Returns whether the database actually has updated since last call.
        private bool getSignalStrengths()
        {
            // Assume watts for now.
            double[] signal_strengths = { 900.0, Math.Sqrt(4500.0), Math.Sqrt(4500.0) };
            _signal_strengths = signal_strengths;
            return true;
        }

        // Calculates the angle in the Kinect frame based on
        // the input signal strengths.
        private double getAngle(double[] signal_strengths)
        {
            double A, B, C, D, E, F;
            A = -2 * ANTENNA_POSITION_0.x + 2 * ANTENNA_POSITION_1.x;
            B = -2 * ANTENNA_POSITION_0.y + 2 * ANTENNA_POSITION_1.y;
            C = 1.0 / signal_strengths[0] - 1 / signal_strengths[1] -
                ANTENNA_POSITION_0.x * ANTENNA_POSITION_0.x + ANTENNA_POSITION_1.x * ANTENNA_POSITION_1.x -
                ANTENNA_POSITION_0.y * ANTENNA_POSITION_0.y + ANTENNA_POSITION_1.y * ANTENNA_POSITION_1.y;
            D = -2 * ANTENNA_POSITION_1.x + 2 * ANTENNA_POSITION_2.x;
            E = -2 * ANTENNA_POSITION_1.y + 2 * ANTENNA_POSITION_2.y;
            F = 1.0 / signal_strengths[1] - 1 / signal_strengths[2] -
                ANTENNA_POSITION_1.x * ANTENNA_POSITION_1.x + ANTENNA_POSITION_2.x * ANTENNA_POSITION_2.x -
                ANTENNA_POSITION_1.y * ANTENNA_POSITION_1.y + ANTENNA_POSITION_2.y * ANTENNA_POSITION_2.y;

            // Sanitize division.
            double x_num = C * E - F * B;
            double x_den = E * A - B * D;
            double y_num = C * D - A * F;
            double y_den = B * D - A * E;
            double angle;

            if (Math.Abs(x_den) < 1e-15)
            {
                Console.WriteLine("Error in x position calculation.");
                angle = 0.0;
                
            }
            else if (Math.Abs(y_den) < 1e-15)
            {
                Console.WriteLine("Error in y position calculation.");
                angle = 0.0;
            }
            else
            {
                double x = x_num / x_den;
                double y = y_num / y_den;
                angle = Math.Atan2(y, x);
            }

            return angle;

        }

        private double getRobustAngle(double[] signal_strengths)
        {
            double a1 = triangulate(ANTENNA_POSITION_0.x, ANTENNA_POSITION_0.y,
                ANTENNA_POSITION_1.x, ANTENNA_POSITION_1.y, Math.Sqrt(signal_strengths[0]),
                Math.Sqrt(signal_strengths[1]));
            double a2 = triangulate(ANTENNA_POSITION_1.x, ANTENNA_POSITION_1.y,
                ANTENNA_POSITION_2.x, ANTENNA_POSITION_2.y, Math.Sqrt(signal_strengths[1]),
                Math.Sqrt(signal_strengths[2]));
            double a3 = triangulate(ANTENNA_POSITION_0.x, ANTENNA_POSITION_0.y,
                ANTENNA_POSITION_2.x, ANTENNA_POSITION_2.y, Math.Sqrt(signal_strengths[0]),
                Math.Sqrt(signal_strengths[2]));
            Console.WriteLine("a1, a2, a3 = {0}, {1}, {2}", a1, a2, a3);
            return (a1 + a2 + a3) / 3.0;

        }

        private double triangulate(double x1, double y1, double x2, double y2, double r1, double r2)
        {
            double dx = x1 - x2;
            double dy = y1 - y2;
            double d = Math.Sqrt(dx * dx + dy * dy); // d = |C1-C2|
            double gamma1 = Math.Acos((r2 * r2 + d * d - r1 * r1) / (2 * r2 * d)); // law of cosines
            Console.WriteLine("gamma1 = {0}", gamma1);

            double d1 = r1 * Math.Cos(gamma1); // basic math in right triangle
            double h = r1 * Math.Sin(gamma1);
            double px = x1 + (x2 - x1) / d * d1;
            double py = y1 + (y2 - y1) / d * d1;
            // (-dy, dx)/d is (C2-C1) normalized and rotated by 90 degrees
            double a1x = px + (-dy) / d * h;
            double a1y = py + (+dx) / d * h;
            double a2x = px - (-dy) / d * h;
            double a2y = py - (+dx) / d * h;

            double angle1 = Math.Atan2(a1y, a1x);
            double angle2 = Math.Atan2(a2y, a2x);

            if ((angle1 > 0.0) && (angle1 < Math.PI)) {
                return angle1;
            }
            else
            {
                return angle2;
            }
        }

        #endregion
    }

}
