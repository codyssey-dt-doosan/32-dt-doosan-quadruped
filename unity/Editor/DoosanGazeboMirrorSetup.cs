using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>Creates Assets/Scenes/Plant_GazeboMirror.unity from Plant_Inspection with Gazebo /odom mirroring enabled.</summary>
public static class DoosanGazeboMirrorSetup
{
    public const string SourceScene = "Assets/Scenes/Plant_Inspection.unity";
    public const string ScenePath = "Assets/Scenes/Plant_GazeboMirror.unity";

    [MenuItem("Doosan/ROS 2/Create Gazebo Mirror Scene")]
    public static void Create()
    {
        var scene = EditorSceneManager.OpenScene(SourceScene);
        var root = GameObject.Find("Spot_Robot");
        if (!root) throw new System.InvalidOperationException("Spot_Robot not found in " + SourceScene);
        var bridge = root.GetComponent<UnityRosBridge>();
        var drive = root.GetComponent<DoosanReferencePatrol>();
        if (!bridge || !drive) throw new System.InvalidOperationException("UnityRosBridge or DoosanReferencePatrol missing on Spot_Robot.");

        bridge.gazeboMirror = true;
        bridge.mirrorRelativeOrigin = true;
        bridge.originAtStart = true;
        bridge.publishLidar = false;
        bridge.publishImu = false;
        bridge.publishCamera = true;
        bridge.cameraHz = 10f;
        drive.gazeboMirror = true;

        EditorSceneManager.SaveScene(scene, ScenePath);
        Debug.Log("Created Gazebo Mirror scene: " + ScenePath);
    }
}
