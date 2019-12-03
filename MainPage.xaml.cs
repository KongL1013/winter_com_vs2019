using System;
using System.Text;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using Windows.ApplicationModel.Core;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;
using DJI.WindowsSDK;
using DJIVideoParser;
using System.Diagnostics;
using System.Collections.Generic;
using Windows.UI.Popups;


namespace DJIWSDKDemo
{
    public class Getsocket
    {
        Socket connectedSocket;
        public Boolean isconnected = false;
        public void ConnectSocket(int port, string server = null)
        {
            server = server ?? "127.0.0.1";
            Socket s = null;
            IPEndPoint ipe = new IPEndPoint(IPAddress.Parse(server), port);
            Socket tempSocket =
                new Socket(ipe.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

            try
            {
                tempSocket.Connect(ipe);
            }
            catch
            {
                Console.WriteLine("检查服务器连接.....");
            }

            if (tempSocket.Connected)
            {
                //添加连接成功的状态
                s = tempSocket;
                connectedSocket = s;
                isconnected = true;
            }
            else
            {
                isconnected = false;
            }
        }

        // This method requests the home page content for the specified server.
        public string SocketSendReceive(int port, string server = null)
        {
            server = server ?? Dns.GetHostName();
            string request = "GET / HTTP/1.1\r\nHost: " + server +
                "\r\nConnection: Close\r\n\r\n";
            Byte[] bytesSent = Encoding.ASCII.GetBytes(request);
            Byte[] bytesReceived = new Byte[256];
            string page = "";
            // Create a socket connection with the specified server and port.
            try
            {
                if (connectedSocket == null)
                    return ("Connection failed");

                // Send request to the server.
                connectedSocket.Send(bytesSent, bytesSent.Length, 0);

                // Receive the server home page content.
                int bytes = 0;
                page = "Default HTML page on " + server + ":\r\n";

                // The following will block until the page is transmitted.
                do
                {
                    bytes = connectedSocket.Receive(bytesReceived, bytesReceived.Length, 0);

                    page = page + Encoding.ASCII.GetString(bytesReceived, 0, bytes);
                }
                while (bytes > 0);
            }
            finally
            {
                // 释放资源
                if (connectedSocket != null) ((IDisposable)connectedSocket).Dispose();
            }

            return page;
        }

        public void SendVideoByte(byte[] buffer)
        {
            if (connectedSocket != null)
            {
                try
                {
                    connectedSocket.Send(buffer, buffer.Length, 0);
                }
                catch
                {
                    isconnected = false;
                }
            }
            else
            {
                isconnected = false;
            }
        }
        public void receiveMsgByte(byte[] receiveBuffer)
        {
            if (connectedSocket != null)
            {
                try
                {
                    connectedSocket.Receive(receiveBuffer, receiveBuffer.Length, 0);
                }
                catch
                {
                    isconnected = false;
                }
            }
            else
            {
                isconnected = false;
            }
        }
    }

    public struct ObstacleDistance
    {
        public float front;
        public float back;
        public float left;
        public float right;
        public float down;
    }

    public struct YoloObject
    {
        public int type;
        public int totalNumber;
        public int index;
        public IntPoint2D leftUpPoint;
        public IntPoint2D rightBottomPoint;
    }
    public sealed partial class MainPage : Page
    {
        /************************************************************************************************************/
        /************************************************ 参数声明 ***************************************************/
        /************************************************************************************************************/

        static double M_PI = Math.PI;
        Boolean useSendSocket = false;                      //是否使用发送socket通信
        Boolean useReceiveSocket = false;                   //是否使用接受socket通信
        private DJIVideoParser.Parser videoParser;
        public int sendSocketPort = 9000;                   //发送 端口号对应python 程序接受端口
        public int receiveSocketPort = 9001;                //接受 端口号对应python 程序发送端口
        public int Picturecount = 0;                        //图片已经接受的帧数
        Getsocket socket_send = new Getsocket();            //Socket 发送通信
        Getsocket socket_receive = new Getsocket();         //Socket 接受通信
        public Velocity3D current_velocity;                 //飞机状态变量 速度
        public double current_abs_velocity;                 //飞机状态变量 总速度
        public LocationCoordinate2D current_location;       //飞机状态变量 坐标
        public DoubleMsg current_altitude;                  //飞机状态变量 高度
        public Attitude current_attitude;                   //飞机状态变量 姿态
        System.DateTime logMsgTime;                         //logmsg 时间
        public static System.DateTime startTime;            //系统启动时间
        VissionDetectionState vissionDetectionState;        //视觉避障信息
        bool landing = false;                               //全局landing flag
        private readonly Stopwatch _sw = new Stopwatch();   //用于计时
        public static string Filename;                      //文件存储名
        Windows.Storage.StorageFolder storageFolder;        //文件存储文件夹
        Windows.Storage.StorageFile sampleFile;             //文件handle
        System.IO.StreamWriter sw;                          //文件流
        System.DateTime inittime;                           //00:00时间
        System.DateTime dtime;                              //log相对时间
        public enum logmsgtype                              //logmsg 类型
        {
            NORMAL = 0,
            WARN = 1,
            ERROR = 2
        }
        public enum FlyDirection                            //飞行方向
        {
            Left,
            Right,
            Forward,
            Back,
            Up,
            Down,
            Clockwiseyaw,
            CounterClockwiseyaw
        }
        Grid debugMsgGrid;                                  //debugmsg grid
        byte[] receiveBuffer = new byte[216];               // 4*54 Bytes
        float[] received_data = new float[54];              // 54 float
        string _dispmsg;

        public ObstacleDistance obstacle_dis = new ObstacleDistance();    //激光距离
        public List<YoloObject> yoloObjects = new List<YoloObject>();      //yolo结果
        public float flyWithLaserSpeed = 0.3f;                              //沿墙飞速度
        public double[] visionObstacleFront = new double[4];                //前视障碍探测 从左往右
        public double[] visionObstacleBack = new double[4];                 //后视障碍探测 从左往右
        int YoloImageSize = 608;

        bool is_following_obstacle = false;





        //bool is_writing_received_data = false;

        //public static string str = "234567";
        //public static byte[] receiveBuffer = System.Text.Encoding.Default.GetBytes(str);

        /************************************************************************************************************/
        /************************************************ 主程序 *****************************************************/
        /************************************************************************************************************/
        public MainPage()
        {
            // 初始化 参数等
            this.InitializeComponent();
            startTime = System.DateTime.Now;
            inittime = Convert.ToDateTime("00:00");
            Filename = startTime.ToString("MM-dd-HH-mm-ss") + ".csv";
            DispLogMsg("UWP UI Initialize start time" + startTime.ToString());
            createFile();
            initialSocket();
            //激活app
            DJISDKManager.Instance.SDKRegistrationStateChanged += Instance_SDKRegistrationEvent;
            DJISDKManager.Instance.RegisterApp("c6710ae96c32f7bb392c697f");
        }

        public async void receiveMsg()
        {
            DispLogMsg("Ready to receive msg....");
            int count = 0;
            bool laserDataFlag = true;
            while (true)
            {
                try
                {
                    if (socket_receive.isconnected)
                    {
                        socket_receive.receiveMsgByte(receiveBuffer);
                        _dispmsg = "";
                        //is_writing_received_data = true;
                        for (int i = 0; i < received_data.Length; i++)
                        {
                            received_data[i] = BitConverter.ToSingle(receiveBuffer, 4 * i);
                            //if (i == 0)
                            //{
                            //    _dispmsg += "左 " +received_data[i].ToString("f3") + " ";
                            //}else if (i == 1)
                            //{
                            //    _dispmsg += "右 " + received_data[i].ToString("f3") + " ";
                            //}
                            //else if (i == 2)
                            //{
                            //    _dispmsg += "前 " + received_data[i].ToString("f3") + " ";
                            //}
                            //else if (i == 3)
                            //{
                            //    _dispmsg += "后 " + received_data[i].ToString("f3") + " ";
                            //}
                            //else
                            //{
                            //    //_dispmsg += received_data[i].ToString() + " ";
                            //}
                            //DispLogMsg(received_data[i].ToString());
                            //count++;
                            //if (count == 4 && laserDataFlag)
                            //{
                            //    _dispmsg += '\n';
                            //    laserDataFlag = false;
                            //    count = 0;
                            //}
                            //count++;
                            //if (count == 5)
                            //{
                            //    _dispmsg += '\n';
                            //    count = 0;
                            //}
                        }


                        //is_writing_received_data = false;
                        //update obstacle_dis and yoloObjects from received_data[]
                        obstacle_dis.front = received_data[2];
                        obstacle_dis.down = received_data[3];
                        obstacle_dis.left = received_data[0];
                        obstacle_dis.right = received_data[1];

                        yoloObjects.Clear();
                        for (int ii = 0; ii < 10; ii++)
                        {
                            YoloObject yoloObject = new YoloObject();
                            yoloObject.type = (int)received_data[ii * 5 + 4];

                            if (yoloObject.type == 9)
                                continue;

                            if (yoloObject.type == 0)
                                break;
                            yoloObject.totalNumber = (int)(received_data[ii * 5 + 4] * 10) % 10;
                            yoloObject.index = (int)(received_data[ii * 5 + 4] * 100) % 10;
                            yoloObject.leftUpPoint = new IntPoint2D { x = (int)received_data[ii * 5 + 5], y = (int)received_data[ii * 5 + 6] };
                            yoloObject.rightBottomPoint = new IntPoint2D { x = (int)received_data[ii * 5 + 7], y = (int)received_data[ii * 5 + 8] };
                            yoloObjects.Add(yoloObject);
                        }

                        _dispmsg += "左 " + obstacle_dis.left.ToString("f3") + " " + "右 " + obstacle_dis.right.ToString("f3") + " "+ "前 " 
                            + obstacle_dis.front.ToString("f3") + " "+ "下 " + obstacle_dis.down.ToString("f3") + " ";

                        string yoloresultstring = "";

                        foreach (var yolotemp in yoloObjects)
                        {
                            yoloresultstring += yolotemp.type.ToString() + " " + yolotemp.totalNumber.ToString() + " " + yolotemp.index.ToString()
                                + " (" + yolotemp.leftUpPoint.x.ToString() + "," + yolotemp.leftUpPoint.y.ToString() + ") "
                                + " (" + yolotemp.rightBottomPoint.x.ToString() + "," + yolotemp.rightBottomPoint.y.ToString() + ") \n";
                        }

                        await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                        {
                            dipYoloDetectionResult.Header = "YoloResultNumber: " + yoloObjects.Count().ToString();
                            dipYoloDetectionResult.Text = yoloresultstring;
                            dipReceiveMsg.Text = "Receved msg：" + _dispmsg;
                                                    });

                    }
                    else
                    {
                        socket_receive.isconnected = false;
                        //DispLogMsg("receive socket connected failed");
                    }
                }
                catch
                {
                    socket_receive.isconnected = false;
                    //DispLogMsg("receive socket connected failed");
                }
            }
        }

        //初始化Socket
        public void initialSocket()
        {
            if (useSendSocket)
            {
                DispLogMsg("connecting socket...");
                socket_send.ConnectSocket(sendSocketPort);
            }
            else
            {
                DispLogMsg("don't use Send socket...");
            }
            if (useReceiveSocket)
            {
                DispLogMsg("connecting socket...");
                socket_receive.ConnectSocket(receiveSocketPort);
            }
            else
            {
                DispLogMsg("don't use Receive socket...");
            }
        }

        //初始化存储文件
        private async void createFile()
        {
            storageFolder = Windows.Storage.ApplicationData.Current.LocalFolder;
            sampleFile = await storageFolder.CreateFileAsync(Filename, Windows.Storage.CreationCollisionOption.ReplaceExisting);
            //await Windows.Storage.FileIO.WriteTextAsync(sampleFile, "Swift as a shadow1,");

            /**************************************************************************************************************/
            /*********************************** 一种读取写入文件方法 *******************************************************/
            /**************************************************************************************************************/

            sw = new StreamWriter(storageFolder.Path + "\\" + Filename, true, Encoding.ASCII);
            DispLogMsg(Filename + ": 文件初始化完成");
            DispLogMsg("路径：" + storageFolder.Path);

            //sw.Write("aaaaa\n");
            //sw.Write("aaaaa,");
            //sw.Flush();
            //sw.Close();

            /************************************************************************************************************/
            /*********************************** 另一种读取文件方法 *******************************************************/
            /************************************************************************************************************/

            //var stream = await sampleFile.OpenAsync(Windows.Storage.FileAccessMode.ReadWrite);
            //using (var outputStream = stream.GetOutputStreamAt(0))
            //{
            //    using (var dataWriter = new Windows.Storage.Streams.DataWriter(outputStream))
            //    {
            //        dataWriter.WriteString("DataWriter has methods to write to various types, such as DataTimeOffset.1,");
            //        await dataWriter.StoreAsync();
            //        await outputStream.FlushAsync();
            //    }
            //}
            //stream.Dispose();
        }

        //log 打印函数
        public async void DispLogMsg(string msg, logmsgtype type = logmsgtype.NORMAL)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                logMsgTime = System.DateTime.Now;
                dtime = inittime + (logMsgTime - startTime);
                string logMsg = "【" + dtime.ToString("mm:ss:ffff") + "】【" + type.ToString() + "】 " + msg + "\n";
                debugMsg.Text += logMsg;
                //Console.WriteLine(logMsg);
                ScrollToEnd();
            });
        }
        //textbox 滑动到最下方
        public void ScrollToEnd()
        {
            debugMsgGrid = (Grid)VisualTreeHelper.GetChild(debugMsg, 0);
            if (debugMsgGrid != null)
            {
                for (var i = 0; i <= VisualTreeHelper.GetChildrenCount(debugMsgGrid) - 1; i++)
                {
                    object obj = VisualTreeHelper.GetChild(debugMsgGrid, 2);
                    if (!(obj is ScrollViewer)) continue;
                    ((ScrollViewer)obj).ChangeView(0.0f, ((ScrollViewer)obj).ExtentHeight, 1.0f);
                    break;
                }
            }
        }

        //延时函数
        private void ShortDelay(double milliseconds)
        {
            _sw.Start();
            while ((_sw.Elapsed).TotalMilliseconds < milliseconds) { }
            _sw.Reset();
        }

        //飞机状态信息回调函数
        private async void ComponentHandingPage_VelocityChanged(object sender, Velocity3D? value)
        {

            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
           {
               if (value != null)
               {
                   current_velocity = value.Value;
                   current_abs_velocity = Math.Sqrt(current_velocity.x * current_velocity.x + current_velocity.y * current_velocity.y);
                   dipFlightVelocityMsg.Text = "NED Velocity Vsum: " + current_abs_velocity.ToString("f2") +
                       " m/s Vx:  " + current_velocity.x.ToString("f1") +
                       "  Vy:  " + current_velocity.y.ToString("f1") +
                       "  Vz:  " + current_velocity.z.ToString("f1");
                   sw.Write(current_velocity.x.ToString() + ",");
                   sw.Write(current_velocity.y.ToString() + ",");
                   sw.Write(current_velocity.z.ToString() + ",");
                   sw.Write(current_abs_velocity.ToString() + "\n");
                   sw.Flush();
               }
           });
        }
        private async void ComponentHandingPage_LocationChanged(object sender, LocationCoordinate2D? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (value != null)
                {
                    current_location = value.Value;
                    double L = 6381372 * M_PI * 2;//地球周长  
                    double W = L;// 平面展开后，x轴等于周长  
                    double H = L / 2;// y轴约等于周长一半  
                    double mill = 2.3;// 米勒投影中的一个常数，范围大约在正负2.3之间  
                    double x = current_location.latitude * M_PI / 180;// 将经度从度数转换为弧度  
                    double y = current_location.longitude * M_PI / 180;// 将纬度从度数转换为弧度  
                    y = 1.25 * Math.Log(Math.Tan(0.25 * M_PI + 0.4 * y));// 米勒投影的转换  
                                                                         // 弧度转为实际距离  
                    x = (W / 2) + (W / (2 * M_PI)) * x;
                    y = (H / 2) - (H / (2 * mill)) * y;
                    //dipFlightLocation2DMsg.Text = "Location2D lat:  " + current_location.latitude.ToString("f15") + "  lon:  " + current_location.longitude.ToString("f15") + "   ";
                    dipFlightLocation2DMsg.Text = "Location2D lat:  " + x.ToString("f15") + "  lon:  " + y.ToString("f15") + "   ";

                }
            });
        }
        private async void ComponentHandingPage_AltitudeChanged(object sender, DoubleMsg? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (value != null)
                {
                    current_altitude = value.Value;
                    dipFlightAltitudeMsg.Text = "Hight from barometer:  " + current_altitude.value.ToString("f1") + "  m";
                }
            });
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>{/*add code here*/});
        }
        private async void ComponentHandingPage_BatteryChanged(object sender, IntMsg? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
            {
                if (value != null)
                {
                    dipBatteryMsg.Text = "battery : " + value.Value.value.ToString();
                    if (value.Value.value<15)
                    {
                        await new MessageDialog("Battery Low!!!").ShowAsync();
                    }
                }
            });
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>{/*add code here*/});
        }
        private async void ComponentHandingPage_AttitudeChanged(object sender, Attitude? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (value != null)
                {
                    current_attitude = value.Value;
                    dipFlightAttitudeMsg.Text = string.Format("Attitude pitch: {0,5:f2}", current_attitude.pitch) +
                        string.Format(" roll : {0,5:f2}", current_attitude.roll) +
                        string.Format(" yaw : {0,5:f2}", current_attitude.yaw);
                }
            });
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>{/*add code here*/});
        }
        private async void ComponentHandingPage_VissionDetectionStateChanged(object sender, VissionDetectionState? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (value != null)
                {
                    vissionDetectionState = value.Value;

                    String obs_dis_string = "";
                    if (vissionDetectionState.position == VisionSensorPosition.NOSE)
                    {
                        obs_dis_string = "Front : ";
                        for (int i = 0; i < vissionDetectionState.detectionSectors.Count(); i++)
                        {
                            obs_dis_string += "   " + vissionDetectionState.detectionSectors[i].obstacleDistanceInMeters.ToString();
                            visionObstacleFront[i] = (vissionDetectionState.detectionSectors[i].obstacleDistanceInMeters);
                        }
                        dipFrontObstacle.Text = obs_dis_string;
                    }
                    else if (vissionDetectionState.position == VisionSensorPosition.TAIL)
                    {
                        obs_dis_string = "Back  : ";
                        for (int i = vissionDetectionState.detectionSectors.Count() - 1; i >= 0; i--)
                        {
                            obs_dis_string += "   " + vissionDetectionState.detectionSectors[i].obstacleDistanceInMeters.ToString();
                            visionObstacleBack[3 - i] = (vissionDetectionState.detectionSectors[i].obstacleDistanceInMeters);
                        }
                        dipBackObstacle.Text = obs_dis_string;
                    }
                }

            });

        }
        private async void ComponentHandingPage_ConnectionChanged(object sender, BoolMsg? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                if (value != null)
                {
                    DispLogMsg("Wifi connect status: " + value.Value.value.ToString());
                    dipWiFiMsg.Text = "Wifi connect status: " + value.Value.value.ToString();
                }
            });
        }

        private async void ComponentHandingPage_VisionAssistedPositioningEnabledChanged(object sender, BoolMsg? value)
        {
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
            {
                if (value != null)
                {
                    if (!value.Value.value)
                    {
                        await new MessageDialog("视觉定位已关闭！！！").ShowAsync();
                    }else
                    {
                        DispLogMsg("视觉定位已开启！！！");
                    }
                }
            });
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>{/*add code here*/});
        }


        //注册函数回调函数，在此添加飞控状态信息变化事件
        private async void Instance_SDKRegistrationEvent(SDKRegistrationState state, SDKError resultCode)
        {
            if (resultCode == SDKError.NO_ERROR)
            {
                //System.Diagnostics.Debug.WriteLine("Register app successfully.");
                //Must in UI thread
                await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
                {
                    activeStatusText.Text = "Register app successfully ! !";
                    SDKRegisButton.Content = "SDKRegistration SUCCESSED";
                    SDKRegisButton.Background = new SolidColorBrush(Windows.UI.Color.FromArgb(204, 16, 249, 16));

                    if (DJISDKManager.Instance.ComponentManager != null)
                    {
                        DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).VissionDetectionStateChanged += ComponentHandingPage_VissionDetectionStateChanged;
                        DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).VelocityChanged += ComponentHandingPage_VelocityChanged;
                        DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AircraftLocationChanged += ComponentHandingPage_LocationChanged;
                        DJISDKManager.Instance.ComponentManager.GetBatteryHandler(0, 0).ChargeRemainingInPercentChanged += ComponentHandingPage_BatteryChanged;
                        DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AltitudeChanged += ComponentHandingPage_AltitudeChanged;
                        DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AttitudeChanged += ComponentHandingPage_AttitudeChanged;
                        DJISDKManager.Instance.ComponentManager.GetWiFiHandler(0, 0).ConnectionChanged += ComponentHandingPage_ConnectionChanged;
                        DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).VisionAssistedPositioningEnabledChanged += ComponentHandingPage_VisionAssistedPositioningEnabledChanged;

                        DispLogMsg("flight status event added");
                    }
                    else
                    {
                        DispLogMsg("ComponentManager NULL", logmsgtype.WARN);
                    }
                    //Raw data and decoded data listener

                    if (videoParser == null)
                    {
                        videoParser = new DJIVideoParser.Parser();
                        videoParser.Initialize(delegate (byte[] data)
                        {
                            //Note: This function must be called because we need DJI Windows SDK to help us to parse frame data.
                            return DJISDKManager.Instance.VideoFeeder.ParseAssitantDecodingInfo(0, data);
                        });
                        //Set the swapChainPanel to display and set the decoded data callback.
                        videoParser.SetSurfaceAndVideoCallback(0, 0, DispVideo, ReceiveDecodedData);
                        DJISDKManager.Instance.VideoFeeder.GetPrimaryVideoFeed(0).VideoDataUpdated += OnVideoPush;
                    }
                    //get the camera type and observe the CameraTypeChanged event.
                    DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).CameraTypeChanged += OnCameraTypeChanged;
                    var type = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).GetCameraTypeAsync();
                    OnCameraTypeChanged(this, type.value);
                });
            }
            else
            {
                await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
               {
                   activeStatusText.Text = "Register app failed: " + resultCode.ToString();
               });

            }
        }

        //图像解码传输
        private async void OnVideoPush(VideoFeed sender, byte[] bytes)
        {
            await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                videoParser.PushVideoData(0, 0, bytes, bytes.Length);
            });

        }
        async void ReceiveDecodedData(byte[] data, int width, int height)
        {
            await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                Picturecount += 1;
                sendMsgCount.Text = "Piture frame: " + Picturecount.ToString() + " Size: " + width.ToString() + " x " + height.ToString();
                //socket
                if (useSendSocket)
                {
                    if (socket_send.isconnected)
                    {
                        Task send_vedio_task = Task.Run(() => socket_send.SendVideoByte(data)); ;
                    }
                    else
                    {
                        ConnectSocketButton.Content = "Socket Connect Failed";
                        DispLogMsg("send Socket Connect Failed", logmsgtype.ERROR);
                        ConnectSocketButton.Background = new SolidColorBrush(Windows.UI.Color.FromArgb(204, 243, 52, 52));
                        useSendSocket = false;
                    }
                }

            });
        }
        private void OnCameraTypeChanged(object sender, CameraTypeMsg? value)
        {
            if (value != null)
            {
                // mavic air
                this.videoParser.SetCameraSensor(AircraftCameraType.Others);
            }
        }

        //按钮点击函数
        private void SDKRegisButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.SDKRegistrationResultCode != SDKError.NO_ERROR)
            {
                DJISDKManager.Instance.RegisterApp("c6710ae96c32f7bb392c697f");
                SDKRegisButton.Content = "SDKRegistration Succeeds";
                SDKRegisButton.Background = new SolidColorBrush(Windows.UI.Color.FromArgb(204, 16, 249, 16));
            }
            else
            {
                DispLogMsg("Already Registrated", logmsgtype.WARN);
            }
        }
        private void ConnectSocketButton_Click(object sender, RoutedEventArgs e)
        {
            // 不连接时执行
            if (!socket_send.isconnected)
            {
                DispLogMsg("Connecting send Socket...");
                socket_send.ConnectSocket(sendSocketPort);
                //Task coonectSendSocketTask = Task.Run(() => socket_send.ConnectSocket(sendSocketPort));
                //判断是否连接成功
                if (socket_send.isconnected)
                {
                    useSendSocket = true;
                    DispLogMsg("send Socket Connected Succeeds");
                }
                else
                {
                    useSendSocket = false;
                    DispLogMsg("send Socket Connect Failed", logmsgtype.ERROR);
                }
            }
            else
            {
                DispLogMsg("Send Socket Already Connected", logmsgtype.WARN);
            }
            // 不连接时执行
            if (!socket_receive.isconnected)
            {
                DispLogMsg("Connecting receive Socket...");
                socket_receive.ConnectSocket(receiveSocketPort);
                //Task coonectReceiveSocketTask = Task.Run(() => socket_receive.ConnectSocket(receiveSocketPort));
                //判断是否连接成功
                if (socket_receive.isconnected)
                {
                    useReceiveSocket = true;
                    DispLogMsg("receive Socket Connected Succeeds");
                }
                else
                {
                    useReceiveSocket = false;
                    DispLogMsg("receive Socket Connect Failed", logmsgtype.ERROR);
                }
            }
            else
            {
                DispLogMsg("Receive Socket Already Connected", logmsgtype.WARN);
            }
            if (socket_receive.isconnected && socket_send.isconnected)
            {
                ConnectSocketButton.Content = "Socket Connect Succeeds";
                ConnectSocketButton.Background = new SolidColorBrush(Windows.UI.Color.FromArgb(204, 16, 249, 16));
            }
            else
            {
                ConnectSocketButton.Content = "Socket Connect Failed";
                ConnectSocketButton.Background = new SolidColorBrush(Windows.UI.Color.FromArgb(204, 243, 52, 52));
            }
        }
        private async void StartTakeoffButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                landing = false;
                var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
                if (res != SDKError.NO_ERROR)
                {
                    DispLogMsg("Send Start takeoff command failed :" + res.ToString(), logmsgtype.ERROR);
                }
                else
                {
                    DispLogMsg("Send Start takeoff command succeeds");
                }
            }
            else
            {
                DispLogMsg("SDK hasn't been activated yet.");
            }

        }
        private async void StopTakeOffButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StopTakeoffAsync();
                if (res != SDKError.NO_ERROR)
                {
                    DispLogMsg("Send Stop takeoff command failed :" + res.ToString(), logmsgtype.ERROR);
                }
                else
                {
                    DispLogMsg("Send Stop takeoff command succeeds");
                }
            }
            else
            {
                DispLogMsg("SDK hasn't been activated yet.");
            }

        }
        private async void StartAutolandButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                //flyTrajectory_Thread.Abort();
                landing = true;
                //摇杆置零
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, 0);
                var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();
                if (res != SDKError.NO_ERROR)
                {
                    DispLogMsg("Send Start autoland command failed :" + res.ToString(), logmsgtype.ERROR);
                }
                else
                {
                    DispLogMsg("Send Start autoland command succeeds");
                }
            }
            else
            {
                DispLogMsg("SDK hasn't been activated yet.");
            }
        }
        private async void StopAutolandButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StopAutoLandingAsync();
                if (res != SDKError.NO_ERROR)
                {
                    DispLogMsg("Send Stop autoland command failed :" + res.ToString(), logmsgtype.ERROR);
                }
                else
                {
                    DispLogMsg("Send Stop autoland command succeeds");
                }
            }
            else
            {
                DispLogMsg("SDK hasn't been activated yet.");
            }

        }
        private void FlyTrajectoryButton_Click(object sender, RoutedEventArgs e)
        {

            DispLogMsg("Fly a Trajectory..");
            Task task = Task.Run(() => flyTrajectory());
        }
        private async void FlyDownButton_Click(object sender, RoutedEventArgs e)
        {
            //DispLogMsg("Fly Down..");
            //Fly(FlyDirection.Down, 0.5);
            ////ShortDelay(1000);
            //await Task.Delay(2200);
            //Hover();
            //await Task.Delay(1000);

            FlyToBaroHeight(3);
        }
        private async void TakeOffAndFlyTrajButton_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                landing = false;
                var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
                if (res != SDKError.NO_ERROR)
                {
                    DispLogMsg("Send Start takeoff command failed :" + res.ToString(), logmsgtype.ERROR);
                }
                else
                {
                    DispLogMsg("Send Start takeoff command succeeds");
                    ShortDelay(3000);
                    Task task = Task.Run(() => flyTrajectory());
                }
            }
            else
            {
                DispLogMsg("SDK hasn't been activated yet.");
            }
        }
        //飞行轨迹测试
        public void flyTrajectory()
        {

            DispLogMsg("fly a Trajectory..");

            //// 找到并对准
            ////Fly(FlyDirection.Left, 0.3f);
            ////while (true)
            ////{
            ////    if (YoloDetection(9))
            ////    {
            ////        break;
            ////    }
            ////}
            ////Hover();
            ////AlignToFrontTarget(9, 0.5f * 0.5f, 0.8f * 0.8f, 0.4f, 0.6f, 0.4f, 0.6f);
            ////DispLogMsg("Adjust to chair Completed !!!!");
            ////Hover();

            ////***************************************
            //// 对准通风口了
            //RotateYaw(90);
            //Fly(FlyDirection.Left, 0.3);
            //while(true)
            //{
            //    if (obstacle_dis.left < 0.8)
            //    {
            //        DispLogMsg("到达左边,距离 " + obstacle_dis.left.ToString());
            //        break;

            //    }
            //}
            //Hover();
            //ShortDelay(100);

            ////**************************
            ////刚进入通风口，左边位置校正
            //MoveToKeepDistanceTo(FlyDirection.Left, 0.5f);

            //Hover();
            //ShortDelay(500);

            ////通风口处下降
            //Fly(FlyDirection.Down, 0.2);
            //while (true)
            //{
            //    if (obstacle_dis.down < 0.55)
            //    {
            //        DispLogMsg("到达down,距离 " + obstacle_dis.down.ToString());

            //        break;

            //    }
            //}
            //Hover();
            //ShortDelay(300);

            //MoveToKeepDistanceTo(FlyDirection.Left, 0.5f);


            ////开始走通风口水平段
            //DispLogMsg("FlyWithLaser 开始.");
            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Left, 0.5f);
            //    ShortDelay(10);
            //    if (obstacle_dis.front < 0.85 || !is_following_obstacle)
            //    {
            //        DispLogMsg("到达front,距离 " + obstacle_dis.front.ToString() + "或is_following_obstacle" + is_following_obstacle.ToString());

            //        break;

            //    }
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();
            //ShortDelay(500);

            ////通风管内部出口处 距离前面0.5m校正
            //MoveToKeepDistanceTo(FlyDirection.Forward, 0.5f);
            //Hover();
            //ShortDelay(500);

            ////左飞 出通风管
            //Fly(FlyDirection.Left, 0.3);
            //while (true)
            //{
            //    if (obstacle_dis.down > 1)
            //    {
            //        DispLogMsg("到达down,距离 " + obstacle_dis.down.ToString()); 
            //        break;
            //    }
            //}
            //Hover();
            //ShortDelay(100);

            //***************************** 通风管完成
            //Yaw左转
            RotateYaw(-90);


            Fly(FlyDirection.Forward, 0.5f);
            ShortDelay(5000);
            Hover();
            ShortDelay(700);


            //while(true)
            //{
            //    if (obstacle_dis.left < 99)
            //    {
            //        DispLogMsg("到达left,距离 " + obstacle_dis.left.ToString());
            //        break;
            //    }
            //}
            //Hover();
            //ShortDelay(100);

            //MoveToKeepDistanceTo(FlyDirection.Left, 1.0f);

            DispLogMsg("FlyWithLaser 开始.");
            while (true)
            {
                FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Left, 1.0f);
                ShortDelay(10);
                if (obstacle_dis.left >3)
                {
                    DispLogMsg("到达left,距离 " + obstacle_dis.left.ToString());
                    break;
                }
            }
            Hover();
            ShortDelay(1000);

            //降低高度
            FlyToBaroHeight(1.2);


            //到达房间左转角,左转90
            RotateYaw(-90);
            //往前飞一点，直到左边激光有值
            Fly(FlyDirection.Forward, 0.5f);
            ShortDelay(5000);
            Hover();
            ShortDelay(700);

            //while (true)
            //{
            //    if (obstacle_dis.left < 99)
            //    {
            //        DispLogMsg("到达left,距离 " + obstacle_dis.left.ToString());
            //        break;
            //    }
            //}

            ////左边校正1m
            //MoveToKeepDistanceTo(FlyDirection.Left, 1.0f);

            //沿墙1m飞
            DispLogMsg("FlyWithLaser 开始.");
            while (true)
            {
                FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Left, 1.0f);
                ShortDelay(10);
                if (obstacle_dis.front < 2)
                {
                    DispLogMsg("到达front,距离 " + obstacle_dis.front.ToString());
                    break;
                }
            }
            DispLogMsg("FlyWithLaser 结束.");

            Hover();
            ShortDelay(1000);
            //左边校正0.5m
            MoveToKeepDistanceTo(FlyDirection.Left, 0.5f);

            //沿墙0.5m飞
            DispLogMsg("FlyWithLaser 开始.");
            while (true)
            {
                FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Left, 0.5f);
                ShortDelay(10);
                if (obstacle_dis.left > 1.5)
                {
                    DispLogMsg("到达,left,距离 " + obstacle_dis.left.ToString());
                    break;
                }
            }
            DispLogMsg("FlyWithLaser 结束.");
            Hover();
            ShortDelay(1000);
            while(obstacle_dis.left < 1)
            {
                Fly(FlyDirection.Back, 0.2f);
                ShortDelay(500);
                Hover();
                ShortDelay(500);
            }
            Fly(FlyDirection.Left, 0.3f);
            while (true)
            {
                if (obstacle_dis.left < 1.1)
                {
                    DispLogMsg("到达left,距离 " + obstacle_dis.left.ToString());
                    break;
                }
            }
            Hover();
            ShortDelay(500);

            


















            //FlyToBaroHeight(1);
            //FlyToBaroHeight(0.4);
            //FlyToBaroHeight(1.2);

            //AlignAndCrossDoor(0.3f * 0.3f, 0.4f * 0.4f, 0.4f, 0.6f, 0.4f, 0.6f);

            //_ = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();

            //Fly(FlyDirection.Left, 0.3f);
            //while (true)    
            //{
            //    if (YoloDetection(9) || obstacle_dis.left < 2)
            //    {
            //        break;
            //    }
            //}
            //Hover();
            //AlignToFrontTarget(9, 0.5f * 0.5f, 0.8f * 0.8f, 0.4f, 0.6f, 0.4f, 0.6f);
            //DispLogMsg("Adjust to chair Completed !!!!");
            //Hover();

            //RotateYaw(-90);

            //Fly(FlyDirection.Right, 0.3f);
            //while (true)
            //{
            //    if (obstacle_dis.front < 1)
            //    {
            //        break;
            //    }
            //}
            //MoveToKeepDistanceTo(FlyDirection.Forward, 0.5f);

            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Right, 0.3f, FlyDirection.Forward, 0.5f);
            //    ShortDelay(10);
            //    if (obstacle_dis.front > 1.5f || obstacle_dis.right < 1f || !is_following_obstacle)
            //        break;
            //}
            //Hover();





            //MoveToKeepDistanceTo(FlyDirection.Left, 0.7f);

            //DispLogMsg("MoveToKeepDistanceTo 结束.");

            //DispLogMsg("FlyWithLaser 开始.");

            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Left, 0.7f);
            //    ShortDelay(10);
            //    if (obstacle_dis.left >1.5f || !is_following_obstacle)
            //        break;
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();

            ////Fly(FlyDirection.Forward, 0.5);
            ////ShortDelay(2300);
            ////Hover();
            ////ShortDelay(500);

            //RotateYaw(-90);

            //Fly(FlyDirection.Forward, 0.5);
            //ShortDelay(2300);
            //Hover();
            //ShortDelay(500);

            //MoveToKeepDistanceTo(FlyDirection.Right, 0.7f);
            //DispLogMsg("MoveToKeepDistanceTo 结束.");

            //DispLogMsg("FlyWithLaser 开始.");

            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Right, 0.7f);
            //    ShortDelay(10);
            //    if (obstacle_dis.front < 0.7f || !is_following_obstacle)
            //        break;
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();

            //RotateYaw(-90);

            //MoveToKeepDistanceTo(FlyDirection.Right, 0.7f);
            //DispLogMsg("MoveToKeepDistanceTo 结束.");

            //DispLogMsg("FlyWithLaser 开始.");

            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Forward, 0.3f, FlyDirection.Right, 0.7f);
            //    ShortDelay(10);
            //    if (obstacle_dis.right > 1.5f || obstacle_dis.front < 1f || !is_following_obstacle)
            //        break;
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();










            /// --------------------------------------------------------------------------------------
            /// ------------------ go through a tunnel      ------------------------------------------
            /// --------------------------------------------------------------------------------------
            //Fly(FlyDirection.Forward, 0.5);
            //ShortDelay(2300);
            //Hover();

            //MoveToKeepDistanceTo(FlyDirection.Left, 0.45f);

            //DispLogMsg("MoveToKeepDistanceTo 结束.");

            //DispLogMsg("FlyWithLaser 开始.");

            //is_following_obstacle = true;

            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Forward,0.3f, FlyDirection.Left, 0.45f);
            //    ShortDelay(10);
            //    if (obstacle_dis.front < 0.85 || !is_following_obstacle)
            //        break;
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();
            ////DispLogMsg("右转弯开始.");
            ////RotateYaw(90);
            ////DispLogMsg("右转弯完成.");


            //MoveToKeepDistanceTo(FlyDirection.Forward, 0.45f);

            //DispLogMsg("MoveToKeepDistanceTo 结束.");

            //DispLogMsg("FlyWithLaser 开始.");

            //is_following_obstacle = true;
            //while (true)
            //{
            //    FlyWithLaser(FlyDirection.Right,0.3f, FlyDirection.Forward, 0.45f);
            //    ShortDelay(10);
            //    if (obstacle_dis.right < 1 || !is_following_obstacle)
            //        break;
            //}
            //DispLogMsg("FlyWithLaser 结束.");
            //Hover();
            //ShortDelay(2000);
            /// --------------------------------------------------------------------------------------
            /// ------------------                 end      ------------------------------------------
            /// --------------------------------------------------------------------------------------


            /// --------------------------------------------------------------------------------------
            /// ------------------ old find a chair and adjust  --------------------------------------
            /// --------------------------------------------------------------------------------------
            //Fly(FlyDirection.Right, 0.5);
            //while (received_data[0] != 1)
            //    ShortDelay(1);
            //DispLogMsg("FIND chair !!!!");

            //Hover();
            //ShortDelay(2000);
            //float[] center_bias = new float[2];
            //float area;

            //while (true)
            //{
            //    center_bias[0] = (received_data[1] + received_data[3]) / 2 - 416 / 2;
            //    center_bias[1] = (received_data[2] + received_data[4]) / 2 - 416 / 2;
            //    area = (received_data[3] - received_data[1]) * (received_data[4] - received_data[2]);
            //    if (Math.Abs(center_bias[0]) < 100 && Math.Abs(center_bias[1]) < 100 && area > 30000 && area < 50000)
            //    {
            //        break;
            //    }
            //    await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            //    {
            //        dipYoloResult.Text = "Yolo_result ";
            //        if (center_bias[0] < 0)   //too left
            //            dipYoloResult.Text += " left  " + center_bias[0].ToString();
            //        else if (center_bias[0] > 0)   //too right
            //            dipYoloResult.Text += " right " + center_bias[0].ToString();
            //        if (center_bias[1] < 0)   //too up
            //            dipYoloResult.Text += " up   " + center_bias[1].ToString();
            //        else if (center_bias[1] > 0)   //too down
            //            dipYoloResult.Text += " down " + center_bias[1].ToString();
            //        if (area < 40000)   //too small
            //            dipYoloResult.Text += " Area small " + area.ToString();
            //        if (area > 40000)   //too big
            //            dipYoloResult.Text += " Area big " + area.ToString();
            //    });

            //    if (center_bias[0] < 0 && !(Math.Abs(center_bias[0]) < 100))   //too left
            //        SmallFlyForAdjust(FlyDirection.Left, 500);
            //    else if (center_bias[0] > 0 && !(Math.Abs(center_bias[0]) < 100))   //too right
            //        SmallFlyForAdjust(FlyDirection.Right, 500);
            //    if (center_bias[1] < 0 && !(Math.Abs(center_bias[1]) < 100))   //too up
            //        SmallFlyForAdjust(FlyDirection.Up, 500);
            //    else if (center_bias[1] > 0 && !(Math.Abs(center_bias[1]) < 100))   //too down
            //        SmallFlyForAdjust(FlyDirection.Down, 500);
            //    if (area < 40000 && !(area > 30000 && area < 50000))   //too small
            //        SmallFlyForAdjust(FlyDirection.Forward, 500);
            //    else if (area > 40000 && !(area > 30000 && area < 50000))   //too big
            //        SmallFlyForAdjust(FlyDirection.Back, 500);
            //}
            //DispLogMsg("Adjust to chair Completed !!!!");

            /// --------------------------------------------------------------------------------------
            /// ------------------ End  --------------------------------------------------------------
            /// --------------------------------------------------------------------------------------
            /// 


            /// --------------------------------------------------------------------------------------
            /// ------------------ find a chair and adjust  ------------------------------------------
            /// --------------------------------------------------------------------------------------

            //Fly(FlyDirection.Left, 0.3f);
            //while (true)
            //{
            //    if (YoloDetection(9) || obstacle_dis.left < 2)
            //    {
            //        break;
            //    }
            //}
            //Hover();
            //AlignToFrontTarget(9, 0.5f * 0.5f, 0.8f * 0.8f, 0.4f, 0.6f, 0.4f, 0.6f);
            //DispLogMsg("Adjust to chair Completed !!!!");
            //Hover();

            /// --------------------------------------------------------------------------------------
            /// ------------------ End  --------------------------------------------------------------
            /// --------------------------------------------------------------------------------------


            /// --------------------------------------------------------------------------------------
            /// ------------------ find a chair and adjust  ------------------------------------------
            /// --------------------------------------------------------------------------------------

            //FlyToLaserHeight(0.5);
            //FlyToLaserHeight(1.2);

            //GimbalRotateSpeed(-20, 6);

            //Fly(FlyDirection.Left, 0.3f);
            //while (true)
            //{
            //    if (YoloDetection(9) || obstacle_dis.left < 2)
            //    {
            //        DispLogMsg("left distance " + obstacle_dis.left.ToString());
            //        break;
            //    }
            //}
            //Hover();
            //if (YoloDetection(9))
            //{
            //    AlignToDownTarget(9, 0, 1, 0.4f, 0.6f, 0.4f, 0.6f);
            //    DispLogMsg("Adjust to chair Completed !!!!");
            //}
            //else
            //{
            //    DispLogMsg("didn't find anything !!!!");
            //}
            //Hover();
            /// --------------------------------------------------------------------------------------
            /// ------------------ End  --------------------------------------------------------------
            /// --------------------------------------------------------------------------------------



            //RotateYaw(90);
            //RotateYaw(90);
            //RotateYaw(90);
            //RotateYaw(90);


            //GimbalRotateAngle(-30);
            //GimbalRotateSpeed(5, 6);
            //GimbalRotateSpeed(-20, 6);

            //GimbalRotateAngle(30);


            //RotateYaw(-90);
            //RotateYaw(-90);






            //Fly(FlyDirection.CounterClockwiseyaw, 0.5);
            //ShortDelay(2000);
            //Hover();
            //ShortDelay(2000);

            //Fly(FlyDirection.Right, 1);
            //Thread.Sleep(2000);
            //Hover();
            //Thread.Sleep(2000);

            //Fly(FlyDirection.Right, 0.6);
            //ShortDelay(5000);
            //Hover();
            //ShortDelay(2000);
            //}

        }

        void SmallFlyForAdjust(FlyDirection direction, float delayMilliseconds)
        {
            Fly(direction, 0.5);
            ShortDelay(delayMilliseconds);
            Hover();
            ShortDelay(delayMilliseconds / 2);
        }

        //悬停按钮
        private void HoverButton_Click(object sender, RoutedEventArgs e)
        {
            Hover();
        }

        //飞行函数
        private void Fly(FlyDirection flydirection, double speedSmallThan1)
        {
            DispLogMsg("fly direction :" + flydirection.ToString() + "speed: " + speedSmallThan1.ToString());
            if (landing)
            {
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, 0);
            }
            else
            {
                if (speedSmallThan1 > 10 || speedSmallThan1 < -10)
                    return;
                switch (flydirection)   //thrust, yaw pitch roll
                {
                    case FlyDirection.Left:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, (float)-speedSmallThan1);
                        break;
                    case FlyDirection.Right:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, (float)speedSmallThan1);
                        break;
                    case FlyDirection.Forward:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, (float)speedSmallThan1, 0);
                        break;
                    case FlyDirection.Back:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, -(float)speedSmallThan1, 0);
                        break;
                    case FlyDirection.Up:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)speedSmallThan1, 0, 0, 0);
                        break;
                    case FlyDirection.Down:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(-(float)speedSmallThan1, 0, 0, 0);
                        break;
                    case FlyDirection.Clockwiseyaw:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, (float)speedSmallThan1, 0, 0);
                        break;
                    case FlyDirection.CounterClockwiseyaw:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, -(float)speedSmallThan1, 0, 0);
                        break;
                }
            }


        }

        private void FlyWithoutOutput(FlyDirection flydirection, double speedSmallThan1)
        {
            if (landing)
            {
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, 0);
            }
            else
            {
                if (speedSmallThan1 > 10 || speedSmallThan1 < -10)
                    return;
                switch (flydirection)   //thrust, yaw pitch roll
                {
                    case FlyDirection.Left:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, (float)-speedSmallThan1);
                        break;
                    case FlyDirection.Right:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, (float)speedSmallThan1);
                        break;
                    case FlyDirection.Forward:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, (float)speedSmallThan1, 0);
                        break;
                    case FlyDirection.Back:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, -(float)speedSmallThan1, 0);
                        break;
                    case FlyDirection.Up:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)speedSmallThan1, 0, 0, 0);
                        break;
                    case FlyDirection.Down:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(-(float)speedSmallThan1, 0, 0, 0);
                        break;
                    case FlyDirection.Clockwiseyaw:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, (float)speedSmallThan1, 0, 0);
                        break;
                    case FlyDirection.CounterClockwiseyaw:
                        DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, -(float)speedSmallThan1, 0, 0);
                        break;
                }
            }


        }

        // 方向可以设置为弧度值
        private void Fly(double flydirection, float speedSmallThan1)
        {
            if (speedSmallThan1 > 2 * Math.PI || speedSmallThan1 < 0)
                return;
            double flght_forword = speedSmallThan1 * Math.Sin(flydirection);
            double flght_right = speedSmallThan1 * Math.Cos(flydirection);
            DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, (float)flght_forword, (float)flght_right);
        }

        private void Hover()
        {
            DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0, 0, 0);

        }

        private void ReceiveMsg_Click(object sender, RoutedEventArgs e)
        {
            Task ReceiveMsgTask = Task.Run(() => receiveMsg());
        }

        private void RotateYaw(float TurnRightAngleInDegree)
        {
            DispLogMsg("开始转" + TurnRightAngleInDegree.ToString() + "°");
            double orignal_yaw = current_attitude.yaw;
            double yaw_to_go;
            yaw_to_go = (orignal_yaw + TurnRightAngleInDegree) > 180 ? orignal_yaw + TurnRightAngleInDegree - 360 : orignal_yaw + TurnRightAngleInDegree;
            yaw_to_go = yaw_to_go < -180 ? yaw_to_go + 360 : yaw_to_go;
            if (TurnRightAngleInDegree > 0)
            {
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, 0.5f, 0, 0);

            }
            else
            {
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, -0.5f, 0, 0);
            }

            while (true)
            {
                double error;
                error = Math.Abs(current_attitude.yaw - yaw_to_go);
                error = error > 180 ? 360 - error : error;
                if (error < 6)
                    break;
                ShortDelay(1);
            }

            int count = 0;
            while (true)
            {
                double error;
                error = yaw_to_go - current_attitude.yaw;
                error = error > 180 ? error - 360 : error;
                error = error < -180 ? error + 360 : error;

                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(0, (float)error * 0.08f, 0, 0);

                if (Math.Abs(error) < 1)
                {
                    count++;

                    if (count > 100)
                        break;
                }
                else
                {
                    count = 0;
                }
                ShortDelay(10);
            }
            Hover();
            ShortDelay(100);
            DispLogMsg("转" + TurnRightAngleInDegree.ToString() + "°好了。");

        }

        private async void CameraFocusButton_Click(object sender, RoutedEventArgs e)
        {
            IntPoint2D focus_point = new IntPoint2D
            {
                x = 1280 / 2,
                y = 960 / 2,
            };
            //CameraFocusModeMsg cameraFocusModeMsg = new CameraFocusModeMsg();
            //cameraFocusModeMsg.value = CameraFocusMode.AFC;
            //var error_msg1 = DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraFocusModeAsync(cameraFocusModeMsg);
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            //{
            //    //debugMsg.Text += error_msg1[]
            //});
            //var error_msg2 = DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraFocusTargetAsync(focus_point);
            //var zoom_support = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).GetCameraOpticalZoomSupportedAsync();
            //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            //{
            //    //debugMsg.Text += error_msg2.ToString();
            //    //debugMsg.Text += zoom_support
            //    if (zoom_support.value != null)
            //    {
            //        debugMsg.Text += zoom_support.value.Value.value.ToString();
            //    }
            //});
            await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraMeteringModeAsync(new CameraMeteringModeMsg { value = CameraMeteringMode.SPOT });
            await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetSpotMeteringTargetPointAsync(focus_point);


        }

        private void GimbalRotateAngle(double pitch)
        {
            GimbalAngleRotation gim = new GimbalAngleRotation
            {

                pitch = pitch,
                roll = 0,
                yaw = 0,
                pitchIgnored = false,
                yawIgnored = true, //true 即忽视他，不转
                rollIgnored = true,
                duration = 1.5, //持续时间，控制转速
            };

            DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0).RotateByAngleAsync(gim);
            ShortDelay(2000);
        }

        private void GimbalRotateSpeed(double pitchSpeed, double rotateSeconds)
        {
            GimbalSpeedRotation sped = new GimbalSpeedRotation
            {
                pitch = pitchSpeed, // The speed on the pitch dimension.
                roll = 0,
                yaw = 0,

            };

            for (int i = 0; i < rotateSeconds * 1000 / 10; i++)
            {
                DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0).RotateBySpeedAsync(sped);
                ShortDelay(10);
            }
            Hover();
            ShortDelay(100);
        }

        private void FlyToLaserHeight(double HeightToGo)
        {
            if (HeightToGo - obstacle_dis.down > 0)
            {
                Fly(FlyDirection.Up, 0.4f);
            }
            else
            {
                Fly(FlyDirection.Down, 0.5f);
            }


            while (true)
            {
                double error;
                error = Math.Abs(HeightToGo - obstacle_dis.down);
                if (error < 0.15)
                    break;
                ShortDelay(10);
            }

            int count = 0;
            while (true)
            {
                double error;
                error = HeightToGo - obstacle_dis.down;
                error = error > 0 ?error * 1.5f : error * 1.5f;

                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)error, 0,0,0);

                if (Math.Abs(error) < 0.1)
                {
                    count++;

                    if (count > 100)
                        break;
                }
                else
                {
                    count = 0;
                }
                ShortDelay(10);
            }
            Hover();
            ShortDelay(100);
        }

        private void FlyToBaroHeight(double HeightToGo)
        {
            if (HeightToGo - current_altitude.value > 0)
            {
                Fly(FlyDirection.Up, 0.4f);
            }
            else
            {
                Fly(FlyDirection.Down, 0.4f);
            }


            while (true)
            {
                double error;
                error = Math.Abs(HeightToGo - current_altitude.value);
                if (error < 0.2)
                    break;
                ShortDelay(10);
            }

            int count = 0;
            while (true)
            {
                double error;
                error = HeightToGo - current_altitude.value;
                error = error > 0 ? error  : error ;

                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)error, 0, 0, 0);

                if (Math.Abs(error) < 0.05)
                {
                    count++;

                    if (count > 100)
                        break;
                }
                else
                {
                    count = 0;
                }
                ShortDelay(10);
            }
            Hover();
            ShortDelay(100);
        }


        private void FlyWithLaser(FlyDirection directionToGo, float goToDirectionSpeed, FlyDirection directionLaser, float distanceToKeep)
        {
            float obstacleDistance = -99999;
            if (directionLaser == FlyDirection.Forward)
                obstacleDistance = obstacle_dis.front;
            else if (directionLaser == FlyDirection.Back)
                obstacleDistance = obstacle_dis.back;
            else if (directionLaser == FlyDirection.Left)
                obstacleDistance = obstacle_dis.left;
            else if (directionLaser == FlyDirection.Right)
                obstacleDistance = obstacle_dis.right;
            else if (directionLaser == FlyDirection.Down)
                obstacleDistance = obstacle_dis.down;

            if (obstacleDistance == 99)
            {
                //DispLogMsg("MoveToKeepDistanceTo激光距离99,已忽略。" + obstacleDistance.ToString());
                FlyWithoutOutput(directionToGo, goToDirectionSpeed);
                ShortDelay(10);
                return;
            }


            //if (Math.Abs(obstacleDistance - distanceToKeep) > 2)
            //{
            //    //await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            //    //{
            //    //    flightStatus.Text = "激光距离与要保持的距离差别大于1米，已退出。";
            //    //});
            //    DispLogMsg("FlyWithLaser激光距离与要保持的距离差别大于2米，已退出。");
            //    is_following_obstacle = false;
            //    return;
            //}

            float flght_forword = 0, flght_right = 0;

            float flght_thrust = 0;

            float error = obstacleDistance - distanceToKeep;   // error负 偏小

            float error_abs = Math.Abs(error);
            float error_sgn = error / error_abs;
            if (error_abs > 0.3)
                error = error_sgn * 0.3f;
            else if (error_abs > 0.2)
                error = error_sgn * 0.2f;
            else if (error_abs > 0.1)
                error = error_sgn * 0.12f;
            else
                error = error_sgn * 0.06f;

            if (directionToGo == FlyDirection.Forward)
            {
                flght_forword = goToDirectionSpeed;
                if (directionLaser == FlyDirection.Left)
                {
                    flght_right = -error;
                }
                else if (directionLaser == FlyDirection.Right)
                {
                    flght_right = error;
                }
                else if (directionLaser == FlyDirection.Forward)
                {
                    flght_forword = error;
                }
                else if (directionLaser == FlyDirection.Back)
                {
                    flght_forword = -error;
                }
            }
            else if (directionToGo == FlyDirection.Back)
            {
                flght_forword = -goToDirectionSpeed;
                if (directionLaser == FlyDirection.Left)
                {
                    flght_right = -error;
                }
                else if (directionLaser == FlyDirection.Right)
                {
                    flght_right = error;
                }
                 
            }
            else if (directionToGo == FlyDirection.Left)
            {
                flght_right = -goToDirectionSpeed;
                if (directionLaser == FlyDirection.Forward)
                {
                    flght_forword = error;
                }
                else if (directionLaser == FlyDirection.Back)
                {
                    flght_forword = -error;
                }
            }
            else if (directionToGo == FlyDirection.Right)
            {
                flght_right = goToDirectionSpeed;
                if (directionLaser == FlyDirection.Forward)
                {
                    flght_forword = error;
                }
                else if (directionLaser == FlyDirection.Back)
                {
                    flght_forword = -error;
                }
            }
            else if (directionToGo == FlyDirection.Down)
            {
                flght_thrust = -0.2f;
                if (directionLaser == FlyDirection.Forward)
                {
                    flght_forword = error;
                }
                else if (directionLaser == FlyDirection.Back)
                {
                    flght_forword = -error;
                }
                else if (directionLaser == FlyDirection.Left)
                {
                    flght_right = -error;
                }
                else if (directionLaser == FlyDirection.Right)
                {
                    flght_right = error;
                }
            }
            else if (directionToGo == FlyDirection.Up)
            {
                flght_thrust = 0.15f;
                if (directionLaser == FlyDirection.Forward)
                {
                    flght_forword = error;
                }
                else if (directionLaser == FlyDirection.Back)
                {
                    flght_forword = -error;
                }
                else if (directionLaser == FlyDirection.Left)
                {
                    flght_right = -error;
                }
                else if (directionLaser == FlyDirection.Right)
                {
                    flght_right = error;
                }
            }


            DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(flght_thrust, 0, flght_forword, flght_right);
            _= CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                flightStatus.Text = "FlyWithLaser:up" + flght_thrust.ToString() +" forward: " + flght_forword.ToString() + " right: " + flght_right.ToString();
            });
            is_following_obstacle = true;
            return;
        }


        private void MoveToKeepDistanceTo(FlyDirection directionLaser, float distanceToKeep)
        {
            DispLogMsg("MoveToKeepDistanceTo。。" + directionLaser.ToString() + "开始接近要保持" + distanceToKeep.ToString());

            while (true)
            {
                float obstacleDistance = -99999;
                if (directionLaser == FlyDirection.Forward)
                    obstacleDistance = obstacle_dis.front;
                else if (directionLaser == FlyDirection.Back)
                    obstacleDistance = obstacle_dis.back;
                else if (directionLaser == FlyDirection.Left)
                    obstacleDistance = obstacle_dis.left;
                else if (directionLaser == FlyDirection.Right)
                    obstacleDistance = obstacle_dis.right;
                if (obstacleDistance == 99)
                {
                    //DispLogMsg("MoveToKeepDistanceTo激光距离99,已忽略。" + obstacleDistance.ToString());
                    Hover();
                    ShortDelay(10);
                    continue;
                }

                //if (Math.Abs(obstacleDistance - distanceToKeep) > 2)
                //{
                //    DispLogMsg("MoveToKeepDistanceTo激光距离与要保持的距离差别大于2米，已退出。" + obstacleDistance.ToString());
                //    return;
                //}

                if (Math.Abs(obstacleDistance - distanceToKeep) < 0.05f)
                {
                    break;
                }

                FlyWithLaser(FlyDirection.Forward, 0.0f, directionLaser, distanceToKeep);
                ShortDelay(10);
            }
            //Hover();
            //ShortDelay(500);
            DispLogMsg("MoveToKeepDistanceTo 到达要保持的距离." + directionLaser.ToString());

            return;
        }

        private void AlignToFrontTarget(int TargetType, float AreaLowerBound, float AreaUpperBound, float centerLeftBound, float centerRightBound, float centerUpBound, float centerDownBound)
        {
            
            float[] center_bias = new float[2];
            float area;

            if(AreaLowerBound > AreaUpperBound || centerLeftBound > centerRightBound || centerUpBound > centerDownBound)
            {
                DispLogMsg("AlignToFrontTarget 参数上下界错误。", logmsgtype.ERROR);
                return;
            }

            while (true)
            {
                List<YoloObject> sameTypeObjects;
                if (!YoloDetection(TargetType, out sameTypeObjects))
                {
                    DispLogMsg("AlignToFrontTarget Can't find target " + TargetType.ToString(), logmsgtype.WARN);
                    Hover();
                    ShortDelay(500);
                    continue;
                }
                YoloObject targetToAlign = new YoloObject();
                targetToAlign = sameTypeObjects[0];



                center_bias[0] = (targetToAlign.leftUpPoint.x + targetToAlign.rightBottomPoint.x) / 2.0f / YoloImageSize;
                center_bias[1] = (targetToAlign.leftUpPoint.y + targetToAlign.rightBottomPoint.y) / 2.0f / YoloImageSize;
                area = (targetToAlign.rightBottomPoint.x - targetToAlign.leftUpPoint.x) *1.0f / YoloImageSize * (targetToAlign.rightBottomPoint.y - targetToAlign.leftUpPoint.y) * 1.0f / YoloImageSize;
                if (center_bias[0] > centerLeftBound && center_bias[0] < centerRightBound
                    && center_bias[1] > centerUpBound && center_bias[1] < centerDownBound
                    && area > AreaLowerBound && area < AreaUpperBound)
                {
                    break;
                }
                _= CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                {
                    dipYoloResult.Text = "Yolo_result ";
                    if (center_bias[0] < centerLeftBound)   //too left
                        dipYoloResult.Text += " left  " + center_bias[0].ToString();
                    else if (center_bias[0] > centerRightBound)   //too right
                        dipYoloResult.Text += " right " + center_bias[0].ToString();
                    if (center_bias[1] < centerUpBound)   //too up
                        dipYoloResult.Text += " up   " + center_bias[1].ToString();
                    else if (center_bias[1] > centerDownBound)   //too down
                        dipYoloResult.Text += " down " + center_bias[1].ToString();
                    if (area < AreaLowerBound)   //too small
                        dipYoloResult.Text += " Area small " + area.ToString();
                    if (area > AreaUpperBound)   //too big
                        dipYoloResult.Text += " Area big " + area.ToString();
                });

                if (center_bias[0] < centerLeftBound)   //too left
                    SmallFlyForAdjust(FlyDirection.Left, 500);
                else if (center_bias[0] > centerRightBound)   //too right
                    SmallFlyForAdjust(FlyDirection.Right, 500);
                if (center_bias[1] < centerUpBound)   //too up
                    SmallFlyForAdjust(FlyDirection.Up, 500);
                else if (center_bias[1] > centerDownBound)   //too down
                    SmallFlyForAdjust(FlyDirection.Down, 500);
                if ( center_bias[0] > centerLeftBound && center_bias[0] < centerRightBound
                    && center_bias[1] > centerUpBound && center_bias[1] < centerDownBound  
                    && area < AreaLowerBound)   //too small
                    SmallFlyForAdjust(FlyDirection.Forward, 500);
                else if (area > AreaUpperBound)   //too big
                    SmallFlyForAdjust(FlyDirection.Back, 500);
            }
            DispLogMsg("Adjust to target Completed !!!!");
        }
        private void AlignToDownTarget(int TargetType, float AreaLowerBound, float AreaUpperBound, float centerLeftBound, float centerRightBound, float centerUpBound, float centerDownBound)
        {

            float[] center_bias = new float[2];
            float area;

            if (AreaLowerBound > AreaUpperBound || centerLeftBound > centerRightBound || centerUpBound > centerDownBound)
            {
                DispLogMsg("AlignToFrontTarget 参数上下界错误。", logmsgtype.ERROR);
                return;
            }

            while (true)
            {
                List<YoloObject> sameTypeObjects;
                if (!YoloDetection(TargetType, out sameTypeObjects))
                {
                    DispLogMsg("AlignToFrontTarget Can't find target " + TargetType.ToString(), logmsgtype.WARN);
                    ShortDelay(500);
                    continue;
                }
                YoloObject targetToAlign = new YoloObject();
                targetToAlign = sameTypeObjects[0];

                center_bias[0] = (targetToAlign.leftUpPoint.x + targetToAlign.rightBottomPoint.x) / 2f / YoloImageSize;
                center_bias[1] = (targetToAlign.leftUpPoint.y + targetToAlign.rightBottomPoint.y) / 2f / YoloImageSize;
                area = (targetToAlign.rightBottomPoint.x - targetToAlign.leftUpPoint.x)*1f / YoloImageSize * (targetToAlign.rightBottomPoint.y - targetToAlign.leftUpPoint.y)*1f / YoloImageSize;
                if (center_bias[0] > centerLeftBound && center_bias[0] < centerRightBound
                    && center_bias[1] > centerUpBound && center_bias[1] < centerDownBound
                    && area > AreaLowerBound && area < AreaUpperBound)
                {
                    break;
                }
                _ = CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                {
                    dipYoloResult.Text = "Yolo_result ";
                    if (center_bias[0] < centerLeftBound)   //too left
                        dipYoloResult.Text += " left  " + center_bias[0].ToString();
                    else if (center_bias[0] > centerRightBound)   //too right
                        dipYoloResult.Text += " right " + center_bias[0].ToString();
                    if (center_bias[1] < centerUpBound)   //too forward
                        dipYoloResult.Text += " forward   " + center_bias[1].ToString();
                    else if (center_bias[1] > centerDownBound)   //too back
                        dipYoloResult.Text += " back " + center_bias[1].ToString();
                    if (area < AreaLowerBound)   //too small
                        dipYoloResult.Text += " Area small " + area.ToString();
                    if (area > AreaUpperBound)   //too big
                        dipYoloResult.Text += " Area big " + area.ToString();
                });

                if (center_bias[0] < centerLeftBound)   //too left
                    SmallFlyForAdjust(FlyDirection.Left, 500);
                else if (center_bias[0] > centerRightBound)   //too right
                    SmallFlyForAdjust(FlyDirection.Right, 500);
                if (center_bias[1] < centerUpBound)   //too forward
                    SmallFlyForAdjust(FlyDirection.Forward, 500);
                else if (center_bias[1] > centerDownBound)   //too back
                    SmallFlyForAdjust(FlyDirection.Back, 500);
                if (area < AreaLowerBound)   //too small
                    SmallFlyForAdjust(FlyDirection.Down, 500);
                else if (area > AreaUpperBound)   //too big
                    SmallFlyForAdjust(FlyDirection.Up, 500);
            }
            DispLogMsg("Adjust to down target Completed !!!!");
        }

        private void AlignAndCrossDoor(float AreaLowerBound, float AreaUpperBound, float centerLeftBound, float centerRightBound, float centerUpBound, float centerDownBound)
        {

            float[] center_bias = new float[2];
            float area;
            int DoorType = 3;
            int TargetType = DoorType;

            if (AreaLowerBound > AreaUpperBound || centerLeftBound > centerRightBound || centerUpBound > centerDownBound)
            {
                DispLogMsg("AlignToFrontTarget 参数上下界错误。", logmsgtype.ERROR);
                return;
            }

            while (true)
            {
                List<YoloObject> sameTypeObjects;
                if (!YoloDetection(TargetType, out sameTypeObjects) || sameTypeObjects.Count() != 2)
                {
                    DispLogMsg("AlignToFrontTarget Can't find 2 targets " + TargetType.ToString(), logmsgtype.WARN);
                    Hover();
                    ShortDelay(500);
                    continue;
                }
                //YoloObject targetToAlign = new YoloObject();

                center_bias[0] = (sameTypeObjects[0].leftUpPoint.x + sameTypeObjects[0].rightBottomPoint.x) / 2.0f / YoloImageSize
                    + (sameTypeObjects[1].leftUpPoint.x + sameTypeObjects[1].rightBottomPoint.x) / 2.0f / YoloImageSize;
                center_bias[0] /= 2f;
                center_bias[1] = (sameTypeObjects[0].leftUpPoint.y + sameTypeObjects[0].rightBottomPoint.y) / 2.0f / YoloImageSize
                    + (sameTypeObjects[1].leftUpPoint.y + sameTypeObjects[1].rightBottomPoint.y) / 2.0f / YoloImageSize;
                center_bias[1] /= 2f;
                area = (sameTypeObjects[0].rightBottomPoint.x - sameTypeObjects[0].leftUpPoint.x) * 1.0f / YoloImageSize 
                    * (sameTypeObjects[0].rightBottomPoint.y - sameTypeObjects[0].leftUpPoint.y) * 1.0f / YoloImageSize
                    +
                    (sameTypeObjects[1].rightBottomPoint.x - sameTypeObjects[1].leftUpPoint.x) * 1.0f / YoloImageSize
                    * (sameTypeObjects[1].rightBottomPoint.y - sameTypeObjects[1].leftUpPoint.y) * 1.0f / YoloImageSize;
                area /= 2f;

                if (center_bias[0] > centerLeftBound && center_bias[0] < centerRightBound
                    && center_bias[1] > centerUpBound && center_bias[1] < centerDownBound
                    && area > AreaLowerBound && area < AreaUpperBound)
                {
                    break;
                }
                _ = CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                {
                    dipYoloResult.Text = "Yolo_result ";
                    if (center_bias[0] < centerLeftBound)   //too left
                        dipYoloResult.Text += " left  " + center_bias[0].ToString();
                    else if (center_bias[0] > centerRightBound)   //too right
                        dipYoloResult.Text += " right " + center_bias[0].ToString();
                    if (center_bias[1] < centerUpBound)   //too up
                        dipYoloResult.Text += " up   " + center_bias[1].ToString();
                    else if (center_bias[1] > centerDownBound)   //too down
                        dipYoloResult.Text += " down " + center_bias[1].ToString();
                    if (area < AreaLowerBound)   //too small
                        dipYoloResult.Text += " Area small " + area.ToString();
                    if (area > AreaUpperBound)   //too big
                        dipYoloResult.Text += " Area big " + area.ToString();
                });

                if (center_bias[0] < centerLeftBound)   //too left
                    SmallFlyForAdjust(FlyDirection.Left, 500);
                else if (center_bias[0] > centerRightBound)   //too right
                    SmallFlyForAdjust(FlyDirection.Right, 500);
                if (center_bias[1] < centerUpBound)   //too up
                    SmallFlyForAdjust(FlyDirection.Up, 500);
                else if (center_bias[1] > centerDownBound)   //too down
                    SmallFlyForAdjust(FlyDirection.Down, 500);
                if (center_bias[0] > centerLeftBound && center_bias[0] < centerRightBound
                    && center_bias[1] > centerUpBound && center_bias[1] < centerDownBound
                    && area < AreaLowerBound)   //too small
                    SmallFlyForAdjust(FlyDirection.Forward, 500);
                else if (area > AreaUpperBound)   //too big
                    SmallFlyForAdjust(FlyDirection.Back, 500);
            }
            DispLogMsg("Adjust to door Completed, wait to go through !!!!");

            while (true)
            {
                if ( obstacle_dis.front > 2)
                {
                    break;
                }
            }
            DispLogMsg("The door is open! ");


            Fly(FlyDirection.Forward, 0.3f);
            while (true)
            {
                if (obstacle_dis.front < 1.2f)
                {
                    break;
                }
            }
            DispLogMsg("已进入！! ");

            Hover();
            ShortDelay(500);



        }


        private bool YoloDetection(int TargetType)
        {
            List<YoloObject> sameTypeObjects = new List<YoloObject>();
            return YoloDetection(TargetType, out sameTypeObjects);
        }
        private bool YoloDetection(int TargetType, out List<YoloObject> sameTypeObjects)
        {
            sameTypeObjects = new List<YoloObject>();
            int same_index = 0;
            bool target_is_found = false;
            for (int i = 0; i < yoloObjects.Count(); i++)
            {
                if (yoloObjects[i].type == TargetType)
                {
                    target_is_found = true;
                    same_index = i;
                    break;
                }
            }
            if(!target_is_found)
            {
                return false;
            }
            else
            {
                for (int i = 0; i < yoloObjects[same_index].totalNumber; i++)
                {
                    sameTypeObjects.Add(yoloObjects[same_index + i]);
                }
                return true;
            }
        }


        private void FlyButtonYawL_Click(object sender, RoutedEventArgs e)
        {
            Task task = Task.Run(()=> RotateYaw(-90));
        }

        private async void FlyButtonUP_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Up, 0.3);
            //ShortDelay(500);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);
        }

        private void FlyButtonYawR_Click(object sender, RoutedEventArgs e)
        {
            Task task = Task.Run(() => RotateYaw(90));
        }

        private async void FlyButtonLeft_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Left, 0.5);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);

        }

        private async void FlyButtonDown_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Down, 0.3);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);

        }

        private async void FlyButtonRight_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Right, 0.5);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);

        }

        private async void FlyButtonForward_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Forward, 0.5);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);
        }

        private async void FlyButtonBack_Click(object sender, RoutedEventArgs e)
        {
            Fly(FlyDirection.Back, 0.5);
            await Task.Delay(500);
            Hover();
            await Task.Delay(500);
        }
    }


}

