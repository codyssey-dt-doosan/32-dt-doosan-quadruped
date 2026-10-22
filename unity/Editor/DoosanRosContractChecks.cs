using System;
using UnityEditor;
using UnityEngine;
using Doosan.Ros2;

public static class DoosanRosContractChecks
{
    static void Check(bool condition,string message){if(!condition)throw new InvalidOperationException("ROS contract: "+message);}
    [MenuItem("Doosan/ROS 2/Check Message Contracts")]
    public static void Run()
    {
        Check(Vector3.Distance(RosCoordinates.ToRos(Vector3.forward),Vector3.right)<1e-6f,"forward axis");
        Check(Vector3.Distance(RosCoordinates.ToRos(Vector3.right),Vector3.down)<1e-6f,"left axis");
        Check(Vector3.Distance(RosCoordinates.ToRos(Vector3.up),Vector3.forward)<1e-6f,"up axis");
        var original=new Vector3(1,2,3);Check(Vector3.Distance(RosCoordinates.ToUnity(RosCoordinates.ToRos(original)),original)<1e-6f,"position roundtrip");
        var left=RosCoordinates.ToRos(Quaternion.Euler(0,-90,0));Check(Math.Abs(RosCoordinates.Yaw(left)-Math.PI/2)<1e-5,"positive ROS yaw turns left");
        var stamp=RosTime.FromSeconds(12.25);Check(stamp.sec==12&&stamp.nanosec==250000000,"nanosecond timestamp");
        string json=JsonUtility.ToJson(new Publication<Odometry>{topic="/odom",msg=new Odometry{header=new Header{stamp=stamp,frame_id="odom"}}});
        Check(json.Contains("\"pose\":{\"pose\"")&&json.Contains("\"nanosec\":250000000"),"odometry JSON shape");
        var bytes=RosCoordinates.PointBytes(new[]{1f,-2f,float.NaN});Check(bytes.Length==12&&bytes[2]==128&&bytes[3]==63&&bytes[7]==192,"little endian point layout");
        Check(float.IsNaN(BitConverter.ToSingle(bytes,8)),"missing lidar returns stay NaN");
        string base64=Convert.ToBase64String(new byte[]{255,0,127});string image=JsonUtility.ToJson(new Publication<Doosan.Ros2.Image>{topic="/camera/image",msg=new Doosan.Ros2.Image{data=base64,width=1,height=1,step=3}});
        Check(image.Contains("\"data\":\"/wB/\""),"base64 image wire format");
        var cmd=JsonUtility.FromJson<CommandEnvelope>("{\"op\":\"publish\",\"topic\":\"/cmd_vel\",\"msg\":{\"linear\":{\"x\":0.3},\"angular\":{\"z\":0.4}}}");
        Check(Math.Abs(cmd.msg.linear.x-.3)<1e-6&&Math.Abs(cmd.msg.angular.z-.4)<1e-6,"Twist decoding");
        Debug.Log("ROS message contracts: 11 checks passed.");
    }
}
