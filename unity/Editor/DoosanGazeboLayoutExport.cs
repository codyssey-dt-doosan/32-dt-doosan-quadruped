using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using Doosan.Ros2;

/// <summary>Exports the plant collision layout (layer 8) as a height grid in the Spot start /odom frame for the Gazebo unity_plant world.</summary>
public static class DoosanGazeboLayoutExport
{
    const float Res=.1f, Margin=6f, Probe=3f, WallSkip=.3f, WallBand=2f, Gap=.1f;
    const int MaxGap=2;
    const short Wall=short.MaxValue, Void=short.MinValue;
    const int PlantMask=1<<8;

    [MenuItem("Doosan/ROS 2/Export Gazebo Layout")]
    public static void Export()
    {
        var robot=GameObject.Find("Spot_Robot");
        if(!robot)throw new InvalidOperationException("Spot_Robot not found in the open scene.");
        var route=GameObject.Find("05_Patrol_Waypoints");
        if(!route||route.transform.childCount==0)throw new InvalidOperationException("05_Patrol_Waypoints with child waypoints not found.");
        Physics.SyncTransforms();
        // Same origin as UnityRosBridge(originAtStart): Spot position and yaw.
        var origin=robot.transform.position;
        var rot=Quaternion.Euler(0,robot.transform.eulerAngles.y,0);
        var inv=Quaternion.Inverse(rot);
        if(!Physics.Raycast(origin+Vector3.up*.5f,Vector3.down,out var g0,3f,PlantMask,QueryTriggerInteraction.Ignore))
            throw new InvalidOperationException("No plant floor (layer 8) under Spot_Robot.");
        float ground=g0.point.y;

        float xmin=0,xmax=0,ymin=0,ymax=0;
        foreach(Transform w in route.transform){
            var r=RosCoordinates.ToRos(inv*(w.position-origin));
            xmin=Mathf.Min(xmin,r.x);xmax=Mathf.Max(xmax,r.x);ymin=Mathf.Min(ymin,r.y);ymax=Mathf.Max(ymax,r.y);
        }
        xmin-=Margin;xmax+=Margin;ymin-=Margin;ymax+=Margin;
        int nx=Mathf.CeilToInt((xmax-xmin)/Res),ny=Mathf.CeilToInt((ymax-ymin)/Res);
        Vector3 At(int i,int j)=>origin+rot*RosCoordinates.ToUnity(new Vector3(xmin+(i+.5f)*Res,ymin+(j+.5f)*Res,0));

        var floors=new float[nx*ny];
        for(int j=0;j<ny;j++)for(int i=0;i<nx;i++){
            var p=At(i,j);
            floors[j*nx+i]=Physics.Raycast(new Vector3(p.x,ground+Probe,p.z),Vector3.down,out var hit,Probe+2f,PlantMask,QueryTriggerInteraction.Ignore)?hit.point.y:float.NaN;
        }

        var cells=new short[nx*ny];
        int walls=0,voids=0,bridged=0;
        var half=new Vector3(Res*.5f,WallBand*.5f,Res*.5f);
        for(int j=0;j<ny;j++)for(int i=0;i<nx;i++){
            int k=j*nx+i;
            float floor=floors[k];
            if(float.IsNaN(floor)){cells[k]=Void;voids++;continue;}
            // A slit in a raised surface (ramp slats) lets the ray fall to the ground below; bridge it from both sides.
            float lifted=Mathf.Max(Bridge(floors,nx,ny,i,j,1,0),Bridge(floors,nx,ny,i,j,0,1));
            if(lifted>floor){floor=lifted;bridged++;}
            var p=At(i,j);
            if(Physics.CheckBox(new Vector3(p.x,floor+WallSkip+WallBand*.5f,p.z),half,rot,PlantMask,QueryTriggerInteraction.Ignore)){cells[k]=Wall;walls++;continue;}
            cells[k]=(short)Mathf.Clamp(Mathf.RoundToInt((floor-ground)*1000f),short.MinValue+1,short.MaxValue-1);
        }

        var bytes=new byte[cells.Length*2];
        Buffer.BlockCopy(cells,0,bytes,0,bytes.Length);
        if(!BitConverter.IsLittleEndian)for(int b=0;b<bytes.Length;b+=2)Array.Reverse(bytes,b,2);
        var json="{\n  \"frame_id\": \"odom\",\n  \"resolution\": "+Res.ToString("R")+",\n  \"origin_x\": "+xmin.ToString("R")+",\n  \"origin_y\": "+ymin.ToString("R")+
            ",\n  \"nx\": "+nx+",\n  \"ny\": "+ny+",\n  \"wall\": "+Wall+",\n  \"void\": "+Void+
            ",\n  \"cells\": \"int16 little-endian, row-major (y rows, x columns), floor height in mm relative to the floor under Spot\",\n  \"data\": \""+Convert.ToBase64String(bytes)+"\"\n}\n";
        var dir=Path.Combine(Path.GetDirectoryName(Application.dataPath),"GazeboLayout");
        Directory.CreateDirectory(dir);
        var path=Path.Combine(dir,"unity_plant_layout.json");
        File.WriteAllText(path,json);
        Debug.Log($"Exported Gazebo layout {nx}x{ny} @ {Res} m (walls {walls}, void {voids}, bridged {bridged}) -> {path}");
    }

    /// <summary>Mean height of the nearest higher floor on both sides (within MaxGap cells, heights within Gap), else -inf.</summary>
    static float Bridge(float[] floors,int nx,int ny,int i,int j,int di,int dj)
    {
        float own=floors[j*nx+i];
        float Side(int s){
            for(int d=1;d<=MaxGap;d++){
                int x=i+s*d*di,y=j+s*d*dj;
                if(x<0||x>=nx||y<0||y>=ny)return float.NaN;
                float v=floors[y*nx+x];
                if(!float.IsNaN(v)&&v>=own+Gap)return v;
            }
            return float.NaN;
        }
        float a=Side(1),b=Side(-1);
        return float.IsNaN(a)||float.IsNaN(b)||Mathf.Abs(a-b)>Gap?float.NegativeInfinity:(a+b)*.5f;
    }
}
