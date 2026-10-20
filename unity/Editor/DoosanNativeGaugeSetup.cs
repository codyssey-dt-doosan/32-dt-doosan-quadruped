using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

public static class DoosanNativeGaugeSetup
{
    [MenuItem("Doosan/ROS 2/Use Original Equipment Gauges")]
    public static void Configure()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        if(UnityEngine.SceneManagement.SceneManager.GetActiveScene().path!=DoosanInspectionSceneSetup.ScenePath)
            EditorSceneManager.OpenScene(DoosanInspectionSceneSetup.ScenePath);
        var robot=GameObject.Find("Spot_Robot");var sensor=robot.GetComponent<UnityPlantInspection>();var bridge=robot.GetComponent<UnityRosBridge>();
        bridge.maxLinearSpeed=2f;
        var machine=GameObject.Find("fac_gastank_1");
        if(!machine)throw new InvalidOperationException("Original equipment not found.");
        var gauges=machine.GetComponentsInChildren<Renderer>().Where(r=>r.name.StartsWith("Gauge_")).OrderBy(r=>r.bounds.center.x).ToArray();
        if(gauges.Length!=3)throw new InvalidOperationException("Expected exactly three original gauges.");
        // Pin a scene-local copy of the original 0-100 bar face texture at full resolution.
        const string face="Assets/DoosanReference/Inspection/OriginalPressureFace.png";
        string original=AssetDatabase.GUIDToAssetPath("9940b9ae2a8224e458cc58704079b82a");
        File.Copy(original,face,true);AssetDatabase.ImportAsset(face,ImportAssetOptions.ForceSynchronousImport);
        var importer=(TextureImporter)AssetImporter.GetAtPath(face);importer.mipmapEnabled=false;importer.streamingMipmaps=false;importer.textureCompression=TextureImporterCompression.Uncompressed;importer.SaveAndReimport();
        var faceTexture=AssetDatabase.LoadAssetAtPath<Texture2D>(face);
        foreach(var renderer in machine.GetComponentsInChildren<Renderer>()){
            var materials=renderer.sharedMaterials;
            for(int i=0;i<materials.Length;i++)if(materials[i]&&materials[i].name.StartsWith("Pressure_01")){
                var m=new Material(materials[i]);m.name="Native_Pressure_01";m.SetTexture("_BaseColorMap",faceTexture);m.SetTexture("_MainTex",faceTexture);
                string path="Assets/DoosanReference/Inspection/"+renderer.name+"_Face_"+i+".mat";AssetDatabase.CreateAsset(m,path);materials[i]=m;
            }
            renderer.sharedMaterials=materials;
        }
        Physics.SyncTransforms();
        foreach(var gauge in gauges){
            var centerPoint=gauge.bounds.center;
            var hits=Physics.RaycastAll(centerPoint+Vector3.forward*.4f,Vector3.back,.8f,1<<8).Where(h=>h.collider.transform.IsChildOf(machine.transform)&&!h.collider.name.StartsWith("Gauge_")).OrderBy(h=>h.distance).ToArray();
            if(hits.Length>0){float depth=hits[0].point.z-centerPoint.z;if(depth>=0&&depth<.12f)gauge.transform.position+=Vector3.forward*(depth+.001f);}
            var material=new Material(gauge.sharedMaterial);material.name="Original_Gauge_Needle_DoubleSided";
            material.SetFloat("_CullMode",0);material.SetFloat("_CullModeForward",0);material.SetFloat("_DoubleSidedEnable",1);material.EnableKeyword("_DOUBLESIDED_ON");
            string asset="Assets/DoosanReference/Inspection/"+gauge.name+".mat";AssetDatabase.CreateAsset(material,asset);gauge.sharedMaterial=material;
        }
        foreach(string name in new[]{"Readable_Gauge_6_2bar","Thermal_Target_95C"}){var old=GameObject.Find(name);if(old)UnityEngine.Object.DestroyImmediate(old);}
        var center=gauges.Select(r=>r.bounds.center).Aggregate(Vector3.zero,(a,b)=>a+b)/3;
        var target=GameObject.Find("Original_Gauge_Inspection_Target")??new GameObject("Original_Gauge_Inspection_Target");
        target.transform.position=center;target.transform.SetParent(machine.transform,true);sensor.inspectionLookTarget=target.transform;sensor.inspectionFov=28;
        (machine.GetComponent<SimulatedHeatSurface>()??machine.AddComponent<SimulatedHeatSurface>()).temperatureC=95;
        var source="Assets/DoosanReference/inspection_route.json";
        var route=JsonUtility.FromJson<DoosanInspectionSceneSetup.InspectionRoute>(File.ReadAllText(source));
        var point=route.waypoints.Single(p=>p.name=="Inspect_Gauges");
        // Retain the already navigable stop in front of this native machine; turn toward its face.
        var inverse=Quaternion.Inverse(Quaternion.Euler(0,bridge.originYawDegrees,0));
        var stop=bridge.originUnity+Quaternion.Euler(0,bridge.originYawDegrees,0)*Doosan.Ros2.RosCoordinates.ToUnity(new Vector3(point.x,point.y,0));
        var direction=Doosan.Ros2.RosCoordinates.ToRos(inverse*(center-stop));point.yaw=Mathf.Atan2(direction.y,direction.x);point.dwell_s=15;
        File.WriteAllText(source,JsonUtility.ToJson(route,true));
        var probe=new Probe{target=center,stop=stop,heading=point.yaw,gauges=gauges.Select(g=>new Gauge{name=g.name,center=g.bounds.center,size=g.bounds.size}).ToArray()};
        File.WriteAllText(Path.Combine(Application.dataPath,"../Logs/OriginalGauges.json"),JsonUtility.ToJson(probe,true));
        EditorUtility.SetDirty(sensor);EditorUtility.SetDirty(machine);
        EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene());AssetDatabase.Refresh();
        Debug.Log("Original equipment gauges selected. Stop="+stop+" target="+center+" heading="+point.yaw);
    }
    [MenuItem("Doosan/ROS 2/Preview Original Gauge Camera")]
    public static void Preview(){Capture(false);}
    public static void ConfigureAndPreview(){Configure();Capture(false);}
    public static void PreviewOtherSide(){Capture(true);}
    static void Capture(bool opposite)
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        EditorSceneManager.OpenScene(DoosanInspectionSceneSetup.ScenePath);
        var robot=GameObject.Find("Spot_Robot");var bridge=robot.GetComponent<UnityRosBridge>();var sensor=robot.GetComponent<UnityPlantInspection>();var cam=bridge.sensorCamera;
        var route=JsonUtility.FromJson<DoosanInspectionSceneSetup.InspectionRoute>(File.ReadAllText("Assets/DoosanReference/inspection_route.json"));var point=route.waypoints.Single(p=>p.inspection);
        var position=robot.transform.position;var rotation=robot.transform.rotation;var camRotation=cam.transform.localRotation;var fov=cam.fieldOfView;var originalTarget=cam.targetTexture;var active=RenderTexture.active;
        var target=new RenderTexture(640,480,24,RenderTextureFormat.ARGB32,RenderTextureReadWrite.sRGB);target.Create();var image=new Texture2D(640,480,TextureFormat.RGB24,false);
        try{
            var stop=bridge.originUnity+Quaternion.Euler(0,bridge.originYawDegrees,0)*Doosan.Ros2.RosCoordinates.ToUnity(new Vector3(point.x,point.y,0));stop.y=.025f;
            if(opposite){stop.z=sensor.inspectionLookTarget.position.z-1.9f;point.yaw=Mathf.PI/2;}
            robot.transform.SetPositionAndRotation(stop,Quaternion.Euler(0,bridge.originYawDegrees-point.yaw*Mathf.Rad2Deg,0));cam.transform.LookAt(sensor.inspectionLookTarget.position);cam.fieldOfView=sensor.inspectionFov;cam.aspect=640f/480;
            cam.targetTexture=target;RenderPipeline.SubmitRenderRequest(cam,new RenderPipeline.StandardRequest{destination=target});RenderTexture.active=target;image.ReadPixels(new Rect(0,0,640,480),0,0);image.Apply();
            File.WriteAllBytes(Path.Combine(Application.dataPath,opposite?"../Logs/OriginalGaugeCameraOther.png":"../Logs/OriginalGaugeCamera.png"),image.EncodeToPNG());
        }finally{robot.transform.SetPositionAndRotation(position,rotation);cam.transform.localRotation=camRotation;cam.fieldOfView=fov;cam.targetTexture=originalTarget;RenderTexture.active=active;target.Release();UnityEngine.Object.DestroyImmediate(target);UnityEngine.Object.DestroyImmediate(image);}
    }
    [Serializable] class Gauge{public string name;public Vector3 center,size;}
    [Serializable] class Probe{public Vector3 target,stop;public float heading;public Gauge[] gauges;}
}
