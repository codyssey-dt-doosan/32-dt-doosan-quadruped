using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Bakes a demonstration route against the imported factory geometry.</summary>
public sealed class DoosanLayoutNavigation
{
    const float Cell=.25f;
    const int Width=101, Height=73;
    readonly bool[] free=new bool[Width*Height];
    Vector3 Position(int id)=>new Vector3((id%Width)*Cell,0,(id/Width)*Cell-9);
    int Id(int x,int z)=>z*Width+x;
    public DoosanLayoutNavigation()
    {
        Physics.SyncTransforms();
        for(int z=0;z<Height;z++)for(int x=0;x<Width;x++){
            int id=Id(x,z);var p=Position(id);
            bool floor=Physics.Raycast(p+Vector3.up*.8f,Vector3.down,out var hit,1.0f,1<<8,QueryTriggerInteraction.Ignore)&&Mathf.Abs(hit.point.y)<.06f;
            free[id]=floor&&!Physics.CheckCapsule(p+Vector3.up*.29f,p+Vector3.up*.5f,.25f,1<<8,QueryTriggerInteraction.Ignore);
        }
    }
    int Nearest(Vector3 target)
    {
        int best=-1;float distance=float.MaxValue;
        for(int i=0;i<free.Length;i++)if(free[i]){float d=(Position(i)-target).sqrMagnitude;if(d<distance){distance=d;best=i;}}
        if(best<0||distance>4)throw new InvalidOperationException("No clear floor near factory waypoint "+target);
        return best;
    }
    public List<Vector3> Path(Vector3 from,Vector3 to)
    {
        int start=Nearest(from),goal=Nearest(to);var open=new List<int>{start};var closed=new bool[free.Length];
        var cost=new float[free.Length];var parent=new int[free.Length];for(int i=0;i<cost.Length;i++){cost[i]=float.PositiveInfinity;parent[i]=-1;}cost[start]=0;
        bool found=false;
        while(open.Count>0){
            int oi=0;float bf=float.MaxValue;
            for(int i=0;i<open.Count;i++){int n=open[i];float f=cost[n]+Vector3.Distance(Position(n),Position(goal))/Cell;if(f<bf){bf=f;oi=i;}}
            int current=open[oi];open.RemoveAt(oi);if(closed[current])continue;closed[current]=true;if(current==goal){found=true;break;}
            int cx=current%Width,cz=current/Width;
            for(int dz=-1;dz<=1;dz++)for(int dx=-1;dx<=1;dx++){
                if(dx==0&&dz==0)continue;int x=cx+dx,z=cz+dz;
                if(x<0||x>=Width||z<0||z>=Height)continue;int next=Id(x,z);if(!free[next]||closed[next])continue;
                if(dx!=0&&dz!=0&&(!free[Id(cx+dx,cz)]||!free[Id(cx,cz+dz)]))continue;
                float g=cost[current]+(dx==0||dz==0?1:1.414214f);
                if(g<cost[next]){cost[next]=g;parent[next]=current;open.Add(next);}
            }
        }
        if(!found)throw new InvalidOperationException("Factory route blocked between "+from+" and "+to);
        var raw=new List<Vector3>();for(int n=goal;n!=-1;n=parent[n])raw.Add(Position(n));raw.Reverse();
        var result=new List<Vector3>();int at=0;
        while(at<raw.Count-1){int furthest=at+1;for(int n=at+2;n<raw.Count;n++){if(!Clear(raw[at],raw[n]))break;furthest=n;}result.Add(raw[furthest]);at=furthest;}
        if(result.Count==0)result.Add(Position(goal));return result;
    }
    static bool Clear(Vector3 a,Vector3 b){var d=b-a;return !Physics.CapsuleCast(a+Vector3.up*.29f,a+Vector3.up*.5f,.25f,d.normalized,out var hit,d.magnitude,1<<8,QueryTriggerInteraction.Ignore);}
}
