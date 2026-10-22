using System;
using System.IO;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using Doosan.Ros2;

public static class DoosanRosSceneSetup
{
    public const string ScenePath="Assets/Scenes/Plant_ROS2.unity";
    [MenuItem("Doosan/ROS 2/Create Connected Scene")]
    public static void Configure()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play before configuring ROS.");
        const string demo="Assets/Scenes/Plant_VideoReference.unity";
        if(!File.Exists(demo))throw new FileNotFoundException("Build the video reference scene first.",demo);
        EditorSceneManager.OpenScene(demo);
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene(),ScenePath,true);
        EditorSceneManager.OpenScene(ScenePath);
        var root=GameObject.Find("Spot_Robot");if(!root)throw new InvalidOperationException("Spot_Robot was not found.");
        var driver=root.GetComponent<DoosanReferencePatrol>();driver.rosControl=true;driver.paused=false;
        var bridge=root.GetComponent<UnityRosBridge>()??root.AddComponent<UnityRosBridge>();
        bridge.controlFromRos=true;bridge.originAtStart=false;bridge.originUnity=root.transform.position;bridge.originYawDegrees=root.transform.eulerAngles.y;
        bridge.serverUrl="ws://127.0.0.1:9090";
        var lens=new GameObject("ROS_RGB_Camera");lens.transform.SetParent(root.transform,false);lens.transform.localPosition=new Vector3(0,.48f,.30f);
        var camera=lens.AddComponent<Camera>();lens.AddComponent<HDAdditionalCameraData>();camera.enabled=false;camera.fieldOfView=60;camera.nearClipPlane=.04f;camera.farClipPlane=30;camera.cullingMask=1<<8;camera.useOcclusionCulling=false;
        bridge.sensorCamera=camera;bridge.worldLayers=1<<8;
        var route=new List<RoutePoint>();var inverse=Quaternion.Inverse(Quaternion.Euler(0,bridge.originYawDegrees,0));
        foreach(var point in driver.route){var p=RosCoordinates.ToRos(inverse*(point.position-bridge.originUnity));route.Add(new RoutePoint{name=point.name,x=p.x,y=p.y,dwell_s=point.name.Contains("Inspect")?2:0});}
        string export=Path.Combine(Application.dataPath,"DoosanReference/ros_route.json");
        File.WriteAllText(export,JsonUtility.ToJson(new Route{frame_id="odom",waypoints=route.ToArray()},true));
        EditorUtility.SetDirty(root);EditorUtility.SetDirty(driver);EditorUtility.SetDirty(bridge);
        EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene());AssetDatabase.SaveAssets();
        DoosanRosContractChecks.Run();
        Debug.Log("ROS 2 scene saved: "+ScenePath+". Start rosbridge, then Play. Unity auto-patrol is disabled in this scene.");
    }
    [Serializable] public class Route{public string frame_id;public RoutePoint[] waypoints;}
    [Serializable] public class RoutePoint{public string name;public float x,y,dwell_s;}
}
