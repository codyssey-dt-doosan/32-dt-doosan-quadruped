using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>Headless Unity CLI validation. Start with -executeMethod DoosanRosBatch.Run (without -quit).</summary>
[InitializeOnLoad]
public static class DoosanRosBatch
{
    const string Running="DoosanRosBatch.Running",Started="DoosanRosBatch.Started";
    static double nextSnapshot;
    static bool hooked;
    static DoosanRosBatch(){if(SessionState.GetBool(Running,false))Hook();}
    public static void Run()
    {
        EditorSceneManager.OpenScene(DoosanRosSceneSetup.ScenePath);
        SessionState.SetBool(Running,true);SessionState.SetString(Started,DateTime.UtcNow.Ticks.ToString());
        var root=GameObject.Find("Spot_Robot");root.GetComponent<DoosanReferencePatrol>().paused=false;
        QualitySettings.vSyncCount=0;Application.targetFrameRate=30;
        Hook();EditorApplication.isPlaying=true;
    }
    static void Hook(){if(hooked)return;hooked=true;EditorApplication.update+=Tick;}
    static void Tick()
    {
        if(!SessionState.GetBool(Running,false))return;
        double elapsed=(DateTime.UtcNow.Ticks-long.Parse(SessionState.GetString(Started,"0")))/(double)TimeSpan.TicksPerSecond;
        var root=GameObject.Find("Spot_Robot");var bridge=root?root.GetComponent<UnityRosBridge>():null;
        if(EditorApplication.isPlaying&&bridge&&elapsed>=nextSnapshot){
            nextSnapshot=elapsed+2;
            Directory.CreateDirectory(Path.Combine(Application.dataPath,"../Logs"));
            File.WriteAllText(Path.Combine(Application.dataPath,"../Logs/UnityRosIntegration.json"),JsonUtility.ToJson(new Snapshot{elapsed=elapsed,connected=bridge.Connected,commands=bridge.AcceptedCommands,odom=bridge.OdometryPublished,images=bridge.ImagesPublished,clouds=bridge.CloudsPublished,position=root.transform.position,commandFresh=bridge.CommandFresh},true));
        }
        if(elapsed>150){
            bool passed=bridge&&bridge.Connected&&bridge.AcceptedCommands>5&&bridge.OdometryPublished>20&&bridge.ImagesPublished>1&&bridge.CloudsPublished>1;
            SessionState.SetBool(Running,false);EditorApplication.Exit(passed?0:2);
        }
    }
    [Serializable] class Snapshot{public double elapsed;public bool connected,commandFresh;public int commands,odom,images,clouds;public Vector3 position;}
}
