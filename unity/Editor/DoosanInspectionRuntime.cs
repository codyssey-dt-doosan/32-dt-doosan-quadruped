using System.IO;
using UnityEditor;
using UnityEngine;

public static class DoosanInspectionRuntime
{
    [MenuItem("Doosan/ROS 2/Inspect Runtime State")]
    public static void Inspect()
    {
        var root=GameObject.Find("Spot_Robot");var b=root?root.GetComponent<UnityRosBridge>():null;
        string text="Playing="+EditorApplication.isPlaying+" Paused="+EditorApplication.isPaused+" Time="+Time.timeAsDouble+" Scale="+Time.timeScale+" Root="+(root&&root.activeInHierarchy)+" Bridge="+(b&&b.enabled)+" Connected="+(b&&b.Connected)+" Odom="+(b?b.OdometryPublished:0);
        File.WriteAllText(Path.Combine(Application.dataPath,"../Logs/InspectionRuntime.txt"),text);Debug.Log(text);
    }
}
