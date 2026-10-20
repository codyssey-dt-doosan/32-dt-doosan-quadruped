using System;
using System.IO;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class DoosanInspectionSceneSetup
{
    public const string ScenePath="Assets/Scenes/Plant_Inspection.unity";
    [MenuItem("Doosan/ROS 2/Create Inspection Scene")]
    public static void Configure()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        EditorSceneManager.OpenScene(DoosanRosSceneSetup.ScenePath);
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene(),ScenePath,true);
        EditorSceneManager.OpenScene(ScenePath);
        var root=GameObject.Find("Spot_Robot");var bridge=root.GetComponent<UnityRosBridge>();
        var inspect=root.AddComponent<UnityPlantInspection>();
        var group=new GameObject("08_ROS_Inspection_Stations");
        const string folder="Assets/DoosanReference/Inspection";Directory.CreateDirectory(folder);
        var leak=GameObject.CreatePrimitive(PrimitiveType.Sphere);leak.name="Simulated_Gas_Source_120ppm";leak.layer=8;
        leak.transform.SetParent(group.transform);leak.transform.position=new Vector3(22.4f,.45f,0);leak.transform.localScale=Vector3.one*.22f;
        var gas=Material("Gas_Source",new Color(.2f,.7f,.25f));AssetDatabase.CreateAsset(gas,folder+"/Gas.mat");leak.GetComponent<Renderer>().sharedMaterial=gas;
        inspect.gasSource=leak.transform;
        AlignGasVisualization();
        bridge.imageWidth=640;bridge.imageHeight=480;bridge.cameraHz=5;
        var points=new List<InspectionPoint>();
        var source=JsonUtility.FromJson<DoosanRosSceneSetup.Route>(File.ReadAllText("Assets/DoosanReference/ros_route.json"));
        foreach(var p in source.waypoints){
            var point=new InspectionPoint{name=p.name,x=p.x,y=p.y,dwell_s=p.dwell_s};
            if(p.name=="Inspect_Gauges"){point.inspection=true;point.yaw=Mathf.PI/2;point.dwell_s=12;}
            points.Add(point);
        }
        File.WriteAllText("Assets/DoosanReference/inspection_route.json",JsonUtility.ToJson(new InspectionRoute{frame_id="odom",waypoints=points.ToArray()},true));
        root.GetComponent<DoosanReferencePatrol>().paused=false;
        EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene());AssetDatabase.SaveAssets();
        DoosanNativeGaugeSetup.Configure();
        DoosanGasMapSetup.Configure();
        Debug.Log("Inspection scene ready: original three equipment gauges, synthetic 95 C surface, simulated 120 ppm gas source.");
    }
    [MenuItem("Doosan/ROS 2/Align Gas Visualization")]
    public static void AlignGasVisualization()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        var root=GameObject.Find("Spot_Robot");var sensor=root?root.GetComponent<UnityPlantInspection>():null;
        var visual=GameObject.Find("Leak_Inspection_Visual_Only")??GameObject.Find("Gas_Leak_Visualization");
        if(!sensor||!sensor.gasSource||!visual)return;
        visual.name="Gas_Leak_Visualization";visual.transform.position=sensor.gasSource.position;
        foreach(var t in visual.GetComponentsInChildren<Transform>(true))t.gameObject.layer=8;
        EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene());
    }
    static Material Material(string name,Color color){var m=new Material(Shader.Find("HDRP/Unlit"));m.name=name;m.SetColor("_UnlitColor",color);return m;}
    [Serializable] public class InspectionPoint{public string name;public float x,y,dwell_s,yaw;public bool inspection;}
    [Serializable] public class InspectionRoute{public string frame_id;public InspectionPoint[] waypoints;}
}
