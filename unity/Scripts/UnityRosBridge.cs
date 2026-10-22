using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Doosan.Ros2;

/// <summary>ROS 2 velocity input and simulated odometry, RGB camera, lidar and IMU output.
/// All scene, rendering, physics and JSON work is executed on Unity's main thread.</summary>
[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(DoosanReferencePatrol))]
public class UnityRosBridge : MonoBehaviour
{
    public string serverUrl="ws://127.0.0.1:9090";
    public bool controlFromRos=true;
    public float commandTimeout=.5f;
    public float maxLinearSpeed=1.5f,maxAngularSpeed=2f;
    public float odometryHz=20,cameraHz=5,lidarHz=5;
    public bool publishCamera=true,publishLidar=true,publishImu=true;
    public Camera sensorCamera;
    public int imageWidth=320,imageHeight=240;
    public Vector3 lidarOffset=new Vector3(0,.56f,0);
    public int lidarColumns=256,lidarRows=16;
    public float lidarVerticalMin=-35,lidarVerticalMax=15,lidarMaxRange=20;
    public LayerMask worldLayers=1<<8;
    public bool originAtStart;
    public Vector3 originUnity;
    public float originYawDegrees=90;

    public string ConnectionStatus=>transport?.Status??"Disconnected";
    public bool Connected=>transport!=null&&transport.Connected;
    public bool CommandFresh=>Connected&&lastCommandTicks!=0&&RosbridgeTransport.Age(lastCommandTicks)<commandTimeout;
    public int AcceptedCommands{get;private set;}
    public int OdometryPublished{get;private set;}
    public int ImagesPublished{get;private set;}
    public int CloudsPublished{get;private set;}
    public float CommandLinear{get;private set;}
    public float CommandAngular{get;private set;}
    RosbridgeTransport transport;
    public event Action<string> MessageReceived;
    DoosanReferencePatrol drive;
    Quaternion inverseOrigin;
    long lastCommandTicks;
    int generation;
    double nextOdom,nextCamera,nextLidar,lastOdomTime;
    Vector3 lastPosition,lastVelocity;
    float lastYaw;
    RenderTexture cameraTarget;
    Texture2D readback;
    bool cameraErrorReported;

    void OnEnable()
    {
        if(!Application.isPlaying)return;
        drive=GetComponent<DoosanReferencePatrol>();drive.rosControl=controlFromRos;drive.ClearRosCommand();
        if(originAtStart){originUnity=transform.position;originYawDegrees=transform.eulerAngles.y;}
        inverseOrigin=Quaternion.Inverse(Quaternion.Euler(0,originYawDegrees,0));
        lastCommandTicks=0;generation=0;CommandLinear=CommandAngular=0;
        AcceptedCommands=OdometryPublished=ImagesPublished=CloudsPublished=0;
        nextOdom=nextCamera=nextLidar=Time.timeAsDouble;lastOdomTime=Time.timeAsDouble;
        lastPosition=transform.position;lastVelocity=Vector3.zero;lastYaw=transform.eulerAngles.y;
        cameraErrorReported=false;
        var ops=new List<string>{JsonUtility.ToJson(new Operation{op="subscribe",topic="/cmd_vel",type="geometry_msgs/msg/Twist"})};
        Register(ops,"/clock","rosgraph_msgs/msg/Clock");Register(ops,"/odom","nav_msgs/msg/Odometry");Register(ops,"/tf","tf2_msgs/msg/TFMessage");
        if(publishImu)Register(ops,"/imu","sensor_msgs/msg/Imu");
        if(publishCamera){Register(ops,"/camera/image","sensor_msgs/msg/Image");Register(ops,"/camera/camera_info","sensor_msgs/msg/CameraInfo");}
        if(publishLidar)Register(ops,"/points","sensor_msgs/msg/PointCloud2");
        if(GetComponent<UnityPlantInspection>()){
            Register(ops,"/thermal/image","sensor_msgs/msg/Image");Register(ops,"/gas/concentration","std_msgs/msg/Float32");
            ops.Add(JsonUtility.ToJson(new Operation{op="subscribe",topic="/inspection/status",type="std_msgs/msg/String"}));
            ops.Add(JsonUtility.ToJson(new Operation{op="subscribe",topic="/mission/status",type="std_msgs/msg/String"}));
        }
        try{transport=new RosbridgeTransport(serverUrl,ops.ToArray());transport.Start();}
        catch(Exception ex){Debug.LogError("ROS connection configuration: "+ex.Message,this);}
        Application.runInBackground=true;
        if(publishCamera&&sensorCamera){
            imageWidth=Mathf.Clamp(imageWidth,32,1280);imageHeight=Mathf.Clamp(imageHeight,24,960);
            cameraTarget=new RenderTexture(imageWidth,imageHeight,24,RenderTextureFormat.ARGB32,RenderTextureReadWrite.sRGB);cameraTarget.Create();
            readback=new Texture2D(imageWidth,imageHeight,TextureFormat.RGB24,false,false);
            sensorCamera.enabled=false;sensorCamera.targetTexture=cameraTarget;sensorCamera.aspect=imageWidth/(float)imageHeight;
        }
    }
    static void Register(List<string> list,string topic,string type)=>list.Add(JsonUtility.ToJson(new Operation{op="advertise",topic=topic,type=type}));
    void Update()
    {
        if(transport==null)return;
        if(generation!=transport.Generation){generation=transport.Generation;lastCommandTicks=0;CommandLinear=CommandAngular=0;}
        while(transport.TryReceive(out var packet)){
            if(RosbridgeTransport.Age(packet.ticks)>commandTimeout)continue;
            try{
                MessageReceived?.Invoke(packet.json);
                var envelope=JsonUtility.FromJson<CommandEnvelope>(packet.json);
                if(envelope==null||envelope.op!="publish"||envelope.topic!="/cmd_vel"||envelope.msg?.linear==null||envelope.msg.angular==null)continue;
                double v=envelope.msg.linear.x,w=envelope.msg.angular.z;
                if(double.IsNaN(v)||double.IsInfinity(v)||double.IsNaN(w)||double.IsInfinity(w))continue;
                CommandLinear=Mathf.Clamp((float)v,-maxLinearSpeed,maxLinearSpeed);CommandAngular=Mathf.Clamp((float)w,-maxAngularSpeed,maxAngularSpeed);
                lastCommandTicks=packet.ticks;AcceptedCommands++;
            }catch(ArgumentException){/* Ignore malformed/non-command rosbridge status messages. */}
        }
        drive.rosControl=controlFromRos;
        if(CommandFresh&&controlFromRos)drive.SetRosCommand(CommandLinear,CommandAngular);else drive.ClearRosCommand();
    }
    void LateUpdate()
    {
        if(!Connected)return;
        double now=Time.timeAsDouble;
        if(now>=nextOdom){nextOdom=now+1/Math.Max(1,odometryHz);PublishMotion(now);}
        if(publishLidar&&now>=nextLidar){nextLidar=now+1/Math.Max(.1,lidarHz);PublishCloud(now);}
        if(publishCamera&&sensorCamera&&cameraTarget&&now>=nextCamera){nextCamera=now+1/Math.Max(.1,cameraHz);PublishImage(now);}
    }
    Header Header(double now,string frame)=>new Header{stamp=RosTime.FromSeconds(now),frame_id=frame};
    Vector3 PositionToRos(Vector3 world)=>RosCoordinates.ToRos(inverseOrigin*(world-originUnity));
    Quaternion OrientationToRos(Quaternion world)=>RosCoordinates.ToRos(inverseOrigin*world);
    public void Publish<T>(string topic,T message){transport?.Publish(topic,JsonUtility.ToJson(new Publication<T>{topic=topic,msg=message}));}
    TransformStamped TF(double now,string parent,string child,Vector3 translation,Quaternion rotation)=>new TransformStamped{header=Header(now,parent),child_frame_id=child,transform=new TransformValue{translation=new Vector3Msg(translation),rotation=new QuaternionMsg(rotation)}};
    void PublishMotion(double now)
    {
        Publish("/clock",new Clock{clock=RosTime.FromSeconds(now)});
        double dt=now-lastOdomTime;Vector3 worldVelocity=dt>1e-6?(transform.position-lastPosition)/(float)dt:Vector3.zero;
        Vector3 bodyVelocity=RosCoordinates.ToRos(transform.InverseTransformDirection(worldVelocity));
        float yaw=transform.eulerAngles.y;float angular=dt>1e-6?-Mathf.DeltaAngle(lastYaw,yaw)*Mathf.Deg2Rad/(float)dt:0;
        var position=PositionToRos(transform.position);var orientation=OrientationToRos(transform.rotation);
        var odom=new Odometry{header=Header(now,"odom")};odom.pose.pose.position=new Vector3Msg(position);odom.pose.pose.orientation=new QuaternionMsg(orientation);
        odom.twist.twist.linear=new Vector3Msg(bodyVelocity);odom.twist.twist.angular.z=angular;
        Publish("/odom",odom);OdometryPublished++;
        var frames=new List<TransformStamped>{TF(now,"map","odom",Vector3.zero,Quaternion.identity),TF(now,"odom","base_link",position,orientation),TF(now,"base_link","lidar",RosCoordinates.ToRos(lidarOffset),Quaternion.identity)};
        if(sensorCamera){
            var offset=transform.InverseTransformPoint(sensorCamera.transform.position);var relative=Quaternion.Inverse(transform.rotation)*sensorCamera.transform.rotation;
            frames.Add(TF(now,"base_link","camera_link",RosCoordinates.ToRos(offset),RosCoordinates.ToRos(relative)));
            frames.Add(TF(now,"camera_link","camera_optical_frame",Vector3.zero,new Quaternion(-.5f,.5f,-.5f,.5f)));
        }
        frames.Add(TF(now,"base_link","imu_link",Vector3.zero,Quaternion.identity));Publish("/tf",new TFMessage{transforms=frames.ToArray()});
        if(publishImu){
            var acceleration=dt>1e-6?(worldVelocity-lastVelocity)/(float)dt:Vector3.zero;
            var specificForce=RosCoordinates.ToRos(transform.InverseTransformDirection(acceleration-Physics.gravity));
            Publish("/imu",new Imu{header=Header(now,"imu_link"),orientation=new QuaternionMsg(orientation),angular_velocity=new Vector3Msg(new Vector3(0,0,angular)),linear_acceleration=new Vector3Msg(specificForce)});
        }
        lastOdomTime=now;lastPosition=transform.position;lastVelocity=worldVelocity;lastYaw=yaw;
    }
    void PublishCloud(double now)
    {
        int columns=Mathf.Clamp(lidarColumns,16,1024),rows=Mathf.Clamp(lidarRows,2,64);var xyz=new float[columns*rows*3];
        var origin=transform.TransformPoint(lidarOffset);
        for(int row=0;row<rows;row++){
            float elevation=Mathf.Lerp(lidarVerticalMin,lidarVerticalMax,row/(float)(rows-1))*Mathf.Deg2Rad;
            for(int col=0;col<columns;col++){
                float azimuth=(-180+360*col/(float)columns)*Mathf.Deg2Rad;
                var local=new Vector3(-Mathf.Sin(azimuth)*Mathf.Cos(elevation),Mathf.Sin(elevation),Mathf.Cos(azimuth)*Mathf.Cos(elevation));
                bool found=Physics.Raycast(origin,transform.TransformDirection(local),out var hit,lidarMaxRange,worldLayers,QueryTriggerInteraction.Ignore)&&hit.distance>=.05f;
                var point=found?RosCoordinates.ToRos(local*hit.distance):new Vector3(float.NaN,float.NaN,float.NaN);
                int at=(row*columns+col)*3;xyz[at]=point.x;xyz[at+1]=point.y;xyz[at+2]=point.z;
            }
        }
        Publish("/points",new PointCloud2{header=Header(now,"lidar"),height=(uint)rows,width=(uint)columns,row_step=(uint)(columns*12),fields=new[]{new PointField{name="x",offset=0},new PointField{name="y",offset=4},new PointField{name="z",offset=8}},data=Convert.ToBase64String(RosCoordinates.PointBytes(xyz)),is_dense=false});CloudsPublished++;
    }
    void PublishImage(double now)
    {
        var previous=RenderTexture.active;
        try{
            if(GraphicsSettings.currentRenderPipeline!=null)RenderPipeline.SubmitRenderRequest(sensorCamera,new RenderPipeline.StandardRequest{destination=cameraTarget});else sensorCamera.Render();
            RenderTexture.active=cameraTarget;readback.ReadPixels(new Rect(0,0,imageWidth,imageHeight),0,0,false);readback.Apply(false);
            var bottomUp=readback.GetRawTextureData<byte>();var topDown=new byte[imageWidth*imageHeight*3];int stride=imageWidth*3;
            for(int y=0;y<imageHeight;y++)for(int x=0;x<stride;x++)topDown[y*stride+x]=bottomUp[(imageHeight-1-y)*stride+x];
            var header=Header(now,"camera_optical_frame");
            Publish("/camera/image",new Doosan.Ros2.Image{header=header,width=(uint)imageWidth,height=(uint)imageHeight,step=(uint)stride,data=Convert.ToBase64String(topDown)});
            double focal=imageHeight/(2*Math.Tan(sensorCamera.fieldOfView*Math.PI/360));double cx=imageWidth/2.0,cy=imageHeight/2.0;
            Publish("/camera/camera_info",new CameraInfo{header=header,width=(uint)imageWidth,height=(uint)imageHeight,k=new[]{focal,0,cx,0,focal,cy,0,0,1},p=new[]{focal,0,cx,0,0,focal,cy,0,0,0,1,0}});ImagesPublished++;
        }catch(Exception ex){if(!cameraErrorReported){cameraErrorReported=true;Debug.LogError("ROS RGB camera capture failed: "+ex.Message,this);}}
        finally{RenderTexture.active=previous;}
    }
    void OnDisable()
    {
        transport?.Dispose();transport=null;lastCommandTicks=0;CommandLinear=CommandAngular=0;
        if(drive)drive.ClearRosCommand();
        if(sensorCamera)sensorCamera.targetTexture=null;
        if(cameraTarget){cameraTarget.Release();Destroy(cameraTarget);cameraTarget=null;}
        if(readback){Destroy(readback);readback=null;}
    }
}
