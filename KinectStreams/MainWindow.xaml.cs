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

namespace KinectStreams
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members


        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        //FirebaseClient _firebase;
        Dictionary<ulong, double[]> _body_positions; 

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
                 //_firebase = new FirebaseClient("https://htn2018-acba7.firebaseio.com");
            }
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

        #endregion
    }

}
