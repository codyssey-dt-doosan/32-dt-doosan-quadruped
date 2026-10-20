using System;
using UnityEngine;

namespace Doosan.Ros2
{
    // ROS 2 wire messages for rosbridge JSON. Byte arrays use rosbridge's base64 encoding.
    [Serializable] public class RosTime { public int sec; public uint nanosec;
        public static RosTime FromSeconds(double value) { value=Math.Max(0,value);var sec=(int)Math.Floor(value);return new RosTime{sec=sec,nanosec=(uint)Math.Min(999999999,Math.Floor((value-sec)*1e9))}; }
    }
    [Serializable] public class Header { public RosTime stamp; public string frame_id; }
    [Serializable] public class Vector3Msg { public double x,y,z; public Vector3Msg(){} public Vector3Msg(Vector3 v){x=v.x;y=v.y;z=v.z;} }
    [Serializable] public class QuaternionMsg { public double x,y,z,w=1; public QuaternionMsg(){} public QuaternionMsg(Quaternion q){x=q.x;y=q.y;z=q.z;w=q.w;} }
    [Serializable] public class Pose { public Vector3Msg position=new Vector3Msg();public QuaternionMsg orientation=new QuaternionMsg(); }
    [Serializable] public class Twist { public Vector3Msg linear=new Vector3Msg();public Vector3Msg angular=new Vector3Msg(); }
    [Serializable] public class PoseWithCovariance { public Pose pose=new Pose();public double[] covariance=new double[36]; }
    [Serializable] public class TwistWithCovariance { public Twist twist=new Twist();public double[] covariance=new double[36]; }
    [Serializable] public class Odometry { public Header header;public string child_frame_id="base_link";public PoseWithCovariance pose=new PoseWithCovariance();public TwistWithCovariance twist=new TwistWithCovariance(); }
    [Serializable] public class TransformValue { public Vector3Msg translation=new Vector3Msg();public QuaternionMsg rotation=new QuaternionMsg(); }
    [Serializable] public class TransformStamped { public Header header;public string child_frame_id;public TransformValue transform=new TransformValue(); }
    [Serializable] public class TFMessage { public TransformStamped[] transforms; }
    [Serializable] public class Clock { public RosTime clock; }
    [Serializable] public class Image { public Header header;public uint height,width;public string encoding="rgb8";public byte is_bigendian;public uint step;public string data; }
    [Serializable] public class RegionOfInterest { public uint x_offset,y_offset,height,width;public bool do_rectify; }
    [Serializable] public class CameraInfo { public Header header;public uint height,width;public string distortion_model="plumb_bob";public double[] d=new double[5];public double[] k=new double[9];public double[] r={1,0,0,0,1,0,0,0,1};public double[] p=new double[12];public uint binning_x,binning_y;public RegionOfInterest roi=new RegionOfInterest(); }
    [Serializable] public class PointField { public string name;public uint offset;public byte datatype=7;public uint count=1; }
    [Serializable] public class PointCloud2 { public Header header;public uint height,width;public PointField[] fields;public bool is_bigendian;public uint point_step=12,row_step;public string data;public bool is_dense; }
    [Serializable] public class Imu { public Header header;public QuaternionMsg orientation;public double[] orientation_covariance={.001,0,0,0,.001,0,0,0,.001};public Vector3Msg angular_velocity;public double[] angular_velocity_covariance={.001,0,0,0,.001,0,0,0,.001};public Vector3Msg linear_acceleration;public double[] linear_acceleration_covariance={.01,0,0,0,.01,0,0,0,.01}; }
    [Serializable] public class CommandEnvelope { public string op,topic;public Twist msg; }
    [Serializable] public class Operation { public string op,topic,type; }
    [Serializable] public class Float32Value { public float data; }
    [Serializable] public class StringValue { public string data; }
    [Serializable] public class StringEnvelope { public string op,topic;public StringValue msg; }
    [Serializable] public class Publication<T> { public string op="publish",topic;public T msg; }

    public static class RosCoordinates
    {
        // Unity right/up/forward -> ROS forward/left/up (REP-103).
        public static Vector3 ToRos(Vector3 v)=>new Vector3(v.z,-v.x,v.y);
        public static Vector3 ToUnity(Vector3 v)=>new Vector3(-v.y,v.z,v.x);
        // A reflection changes the sign of the quaternion's axial-vector part.
        public static Quaternion ToRos(Quaternion q)=>new Quaternion(-q.z,q.x,-q.y,q.w).normalized;
        public static double Yaw(Quaternion q)=>Math.Atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        public static byte[] PointBytes(float[] xyz){var bytes=new byte[xyz.Length*4];Buffer.BlockCopy(xyz,0,bytes,0,bytes.Length);if(!BitConverter.IsLittleEndian)for(int i=0;i<bytes.Length;i+=4)Array.Reverse(bytes,i,4);return bytes;}
    }
}
