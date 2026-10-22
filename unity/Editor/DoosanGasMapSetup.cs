using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

/// <summary>Adds the supplied tank's leak station to the integrated inspection scene.</summary>
public static class DoosanGasMapSetup
{
    const string Folder="Assets/DoosanReference/GasLeak";
    [MenuItem("Doosan/ROS 2/Add Gas Tank Leak Map")]
    public static void Configure()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        EditorSceneManager.OpenScene(DoosanInspectionSceneSetup.ScenePath);
        var robot=GameObject.Find("Spot_Robot");var sensor=robot.GetComponent<UnityPlantInspection>();
        var machine=GameObject.Find("fac_gastank_1");
        if(!machine)throw new InvalidOperationException("Import Modeling/object/fac_gastank.unitypackage first.");
        Directory.CreateDirectory(Folder);AssetDatabase.Refresh();
        // Keep the original imported tank and its three calibrated camera targets intact.
        var old=GameObject.Find("09_Gas_Tank_Leak_Zone");if(old)UnityEngine.Object.DestroyImmediate(old);
        foreach(string name in new[]{"Simulated_Gas_Source_120ppm","Gas_Leak_Visualization","Leak_Inspection_Visual_Only"}){
            var go=GameObject.Find(name);if(go)UnityEngine.Object.DestroyImmediate(go);
        }
        var zone=new GameObject("09_Gas_Tank_Leak_Zone");zone.transform.SetParent(machine.transform,true);
        Physics.SyncTransforms();
        // Attach the outlet to the east end of the actual tank mesh, rather than empty floor.
        var mesh=machine.GetComponent<MeshCollider>();
        if(!mesh||!mesh.Raycast(new Ray(new Vector3(20,1.3f,1),Vector3.left),out var hit,6))
            throw new InvalidOperationException("The east tank wall could not be located.");
        var outlet=hit.point+Vector3.right*.04f;
        var source=new GameObject("Tank_Outlet_Gas_Source_120ppm");source.transform.SetParent(zone.transform);source.transform.position=outlet;
        sensor.gasSource=source.transform;sensor.gasPeakPpm=120;sensor.gasSigma=1.25f;
        var steel=Lit("Outlet_Steel",new Color(.18f,.23f,.25f),.6f);
        var orange=Unlit("Warning_Amber",new Color(1,.42f,.035f));
        var red=Unlit("Alarm_Red",new Color(.95f,.09f,.035f));
        // A short flange and valve show exactly where the demonstration leak originates.
        Cylinder("Tank_Outlet_Flange",zone.transform,outlet-Vector3.right*.025f,.12f,.035f,steel);
        Cylinder("Tank_Outlet_Nozzle",zone.transform,outlet+Vector3.right*.1f,.045f,.12f,steel);
        Cylinder("Valve_Wheel",zone.transform,outlet+new Vector3(.07f,.17f,0),.14f,.018f,red);
        var plume=(GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/effect/gas_spurt_1.prefab"));
        plume.name="Gas_Leak_Visualization";plume.transform.SetParent(source.transform,false);plume.transform.localPosition=Vector3.right*.22f;plume.transform.localRotation=Quaternion.LookRotation(new Vector3(1,.2f,0));
        var ps=plume.GetComponent<ParticleSystem>();ps.Stop(true,ParticleSystemStopBehavior.StopEmittingAndClear);
        var main=ps.main;main.loop=true;main.playOnAwake=true;main.duration=5;main.startLifetime=new ParticleSystem.MinMaxCurve(1.4f,2.2f);
        main.startSpeed=new ParticleSystem.MinMaxCurve(.65f,1.1f);main.startSize=new ParticleSystem.MinMaxCurve(.17f,.32f);main.maxParticles=400;
        main.simulationSpace=ParticleSystemSimulationSpace.World;main.startColor=Color.white;main.gravityModifier=-.015f;main.cullingMode=ParticleSystemCullingMode.AlwaysSimulate;
        var emission=ps.emission;emission.enabled=true;emission.rateOverTime=55;
        var shape=ps.shape;shape.enabled=true;shape.shapeType=ParticleSystemShapeType.Cone;shape.radius=.025f;shape.angle=13;
        var size=ps.sizeOverLifetime;size.enabled=true;size.size=new ParticleSystem.MinMaxCurve(1,new AnimationCurve(new Keyframe(0,.25f),new Keyframe(1,3)));
        var noise=ps.noise;noise.enabled=true;noise.strength=.12f;noise.frequency=.6f;noise.scrollSpeed=.35f;
        var material=Unlit("Gas_Plume",new Color(.2f,.85f,.25f,.4f));
        material.SetTexture("_UnlitColorMap",SoftParticle());material.SetFloat("_SurfaceType",1);material.SetFloat("_BlendMode",0);
        material.SetFloat("_DoubleSidedEnable",1);HDMaterial.ValidateMaterial(material);material.renderQueue=3000;EditorUtility.SetDirty(material);
        var renderer=ps.GetComponent<ParticleSystemRenderer>();renderer.sharedMaterial=material;renderer.renderMode=ParticleSystemRenderMode.Billboard;renderer.maxParticleSize=1;
        float alarmRadius=sensor.gasSigma*Mathf.Sqrt(2*Mathf.Log(sensor.gasPeakPpm/50));
        float cautionRadius=sensor.gasSigma*Mathf.Sqrt(2*Mathf.Log(sensor.gasPeakPpm/20));
        Ring(zone.transform,"50ppm_Alarm_Boundary",outlet,alarmRadius,red);
        Ring(zone.transform,"20ppm_Caution_Boundary",outlet,cautionRadius,orange);
        Sign(zone.transform,outlet+new Vector3(.12f,.95f,0));
        foreach(var t in zone.GetComponentsInChildren<Transform>(true))t.gameObject.layer=8;
        Physics.SyncTransforms();
        var route=UpdateRoute(robot.GetComponent<UnityRosBridge>(),outlet);
        var view=SceneView.lastActiveSceneView;if(view)view.LookAtDirect(outlet,Quaternion.Euler(24,235,0),7);
        EditorUtility.SetDirty(sensor);EditorSceneManager.MarkSceneDirty(UnityEngine.SceneManagement.SceneManager.GetActiveScene());
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene());AssetDatabase.SaveAssets();
        Directory.CreateDirectory("Logs");File.WriteAllText("Logs/GasMap.json",JsonUtility.ToJson(new Report{source=outlet,alarmRadius=alarmRadius,cautionRadius=cautionRadius,waypoints=route.waypoints.Length,providedTank=PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(machine),providedEffect=PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(plume)},true));
        Debug.Log("Gas tank leak map ready: outlet="+outlet+", 50 ppm radius="+alarmRadius+", collision-checked approach route.");
    }
    static DoosanInspectionSceneSetup.InspectionRoute UpdateRoute(UnityRosBridge bridge,Vector3 outlet)
    {
        string path="Assets/DoosanReference/inspection_route.json";
        var route=JsonUtility.FromJson<DoosanInspectionSceneSetup.InspectionRoute>(File.ReadAllText(path));
        var points=route.waypoints.Where(p=>!p.name.StartsWith("Gas_")).ToList();
        int east=points.FindIndex(p=>p.name=="East_Aisle");
        var nav=new DoosanLayoutNavigation();
        var approach=nav.Path(World(bridge,points[east-1]),new Vector3(outlet.x+1.1f,0,outlet.z+.15f));
        var inserted=new List<DoosanInspectionSceneSetup.InspectionPoint>();
        for(int i=0;i<approach.Count;i++)inserted.Add(Point(bridge,i==approach.Count-1?"Gas_Leak_Approach":"Gas_Transit_"+i,approach[i]));
        // Also check the normal continuation, even though a leak causes an earlier emergency return.
        var exit=nav.Path(approach.Last(),World(bridge,points[east]));
        for(int i=0;i<exit.Count-1;i++)inserted.Add(Point(bridge,"Gas_Exit_"+i,exit[i]));
        points.InsertRange(east,inserted);route.waypoints=points.ToArray();File.WriteAllText(path,JsonUtility.ToJson(route,true));
        // Keep the visible editor route consistent with the route consumed by ROS.
        var group=GameObject.Find("05_Patrol_Waypoints");
        foreach(var t in group.GetComponentsInChildren<Transform>().Where(t=>t!=group.transform).ToArray())UnityEngine.Object.DestroyImmediate(t.gameObject);
        var transforms=new List<Transform>();foreach(var p in route.waypoints){var go=new GameObject(p.name);go.transform.SetParent(group.transform);go.transform.position=World(bridge,p);go.transform.position=new Vector3(go.transform.position.x,0,go.transform.position.z);transforms.Add(go.transform);}
        bridge.GetComponent<DoosanReferencePatrol>().route=transforms.ToArray();return route;
    }
    static Vector3 World(UnityRosBridge b,DoosanInspectionSceneSetup.InspectionPoint p)=>b.originUnity+Quaternion.Euler(0,b.originYawDegrees,0)*Doosan.Ros2.RosCoordinates.ToUnity(new Vector3(p.x,p.y,0));
    static DoosanInspectionSceneSetup.InspectionPoint Point(UnityRosBridge b,string name,Vector3 world){var p=Doosan.Ros2.RosCoordinates.ToRos(Quaternion.Inverse(Quaternion.Euler(0,b.originYawDegrees,0))*(world-b.originUnity));return new DoosanInspectionSceneSetup.InspectionPoint{name=name,x=p.x,y=p.y};}
    static Material Unlit(string name,Color color){var m=new Material(Shader.Find("HDRP/Unlit"));m.SetColor("_UnlitColor",color);return Save(m,name);}
    static Material Lit(string name,Color color,float metal){var m=new Material(Shader.Find("HDRP/Lit"));m.SetColor("_BaseColor",color);m.SetFloat("_Metallic",metal);m.SetFloat("_Smoothness",.4f);HDMaterial.ValidateMaterial(m);return Save(m,name);}
    static Material Save(Material m,string name){string path=Folder+"/"+name+".mat";var old=AssetDatabase.LoadAssetAtPath<Material>(path);if(old){EditorUtility.CopySerialized(m,old);UnityEngine.Object.DestroyImmediate(m);return old;}m.name=name;AssetDatabase.CreateAsset(m,path);return m;}
    static Texture2D SoftParticle(){string path=Folder+"/GasSoftParticle.png";var t=new Texture2D(64,64,TextureFormat.RGBA32,false);for(int y=0;y<64;y++)for(int x=0;x<64;x++){float d=new Vector2((x-31.5f)/31.5f,(y-31.5f)/31.5f).magnitude;t.SetPixel(x,y,new Color(1,1,1,Mathf.Pow(Mathf.Clamp01(1-d),2)));}t.Apply();File.WriteAllBytes(path,t.EncodeToPNG());UnityEngine.Object.DestroyImmediate(t);AssetDatabase.ImportAsset(path,ImportAssetOptions.ForceSynchronousImport);var imp=(TextureImporter)AssetImporter.GetAtPath(path);imp.alphaIsTransparency=true;imp.wrapMode=TextureWrapMode.Clamp;imp.SaveAndReimport();return AssetDatabase.LoadAssetAtPath<Texture2D>(path);}
    static void Cylinder(string name,Transform parent,Vector3 p,float radius,float length,Material m){var go=GameObject.CreatePrimitive(PrimitiveType.Cylinder);go.name=name;go.transform.SetParent(parent);go.transform.SetPositionAndRotation(p,Quaternion.Euler(0,0,90));go.transform.localScale=new Vector3(radius*2,length,radius*2);go.GetComponent<Renderer>().sharedMaterial=m;}
    static void Ring(Transform parent,string name,Vector3 source,float radius,Material m){var root=new GameObject(name);root.transform.SetParent(parent);for(int i=0;i<72;i++){if(i%3==2)continue;float a=i*2*Mathf.PI/72;var p=new Vector3(source.x+Mathf.Cos(a)*radius,.014f,source.z+Mathf.Sin(a)*radius);var go=GameObject.CreatePrimitive(PrimitiveType.Cube);go.name="Boundary_Dash";go.transform.SetParent(root.transform);go.transform.SetPositionAndRotation(p,Quaternion.Euler(0,-a*Mathf.Rad2Deg,0));go.transform.localScale=new Vector3(.07f,.012f,2*Mathf.PI*radius/72*.85f);UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());go.GetComponent<Renderer>().sharedMaterial=m;}}
    static void Sign(Transform parent,Vector3 p){var texture=AssetDatabase.LoadAssetAtPath<Texture2D>(Folder+"/GasWarning.png");if(!texture)throw new InvalidOperationException("GasWarning.png missing.");var m=Unlit("Warning_Sign",Color.white);m.SetTexture("_UnlitColorMap",texture);m.SetFloat("_DoubleSidedEnable",1);HDMaterial.ValidateMaterial(m);var go=GameObject.CreatePrimitive(PrimitiveType.Quad);go.name="Gas_Leak_Warning_Sign";go.transform.SetParent(parent);go.transform.SetPositionAndRotation(p,Quaternion.Euler(0,-90,0));go.transform.localScale=new Vector3(1.3f,.65f,1);go.GetComponent<Renderer>().sharedMaterial=m;}
    [MenuItem("Doosan/ROS 2/Preview Gas Tank Map")]
    public static void Capture()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play first.");
        EditorSceneManager.OpenScene(DoosanInspectionSceneSetup.ScenePath);var sensor=GameObject.Find("Spot_Robot").GetComponent<UnityPlantInspection>();
        var source=sensor.gasSource.position;var ps=GameObject.Find("Gas_Leak_Visualization").GetComponent<ParticleSystem>();ps.Simulate(3,true,true,false);
        var go=new GameObject("Gas_Map_Preview");var cam=go.AddComponent<Camera>();go.AddComponent<HDAdditionalCameraData>();cam.fieldOfView=55;cam.nearClipPlane=.04f;cam.farClipPlane=150;cam.cullingMask=1<<8;
        try{CaptureView(cam,source+new Vector3(7,4,6),source+new Vector3(-2,0,0),"GasTankMap.png");CaptureView(cam,source+new Vector3(3,1.2f,3),source+new Vector3(-.6f,.25f,0),"GasTankLeak.png");}
        finally{UnityEngine.Object.DestroyImmediate(go);ps.Stop(true,ParticleSystemStopBehavior.StopEmittingAndClear);}
    }
    static void CaptureView(Camera cam,Vector3 position,Vector3 target,string name){cam.transform.position=position;cam.transform.LookAt(target);var rt=new RenderTexture(1280,720,24,RenderTextureFormat.ARGB32,RenderTextureReadWrite.sRGB);rt.Create();var texture=new Texture2D(1280,720,TextureFormat.RGB24,false);var active=RenderTexture.active;try{cam.aspect=1280f/720;cam.targetTexture=rt;RenderPipeline.SubmitRenderRequest(cam,new RenderPipeline.StandardRequest{destination=rt});RenderTexture.active=rt;texture.ReadPixels(new Rect(0,0,1280,720),0,0);texture.Apply();Directory.CreateDirectory("Logs");File.WriteAllBytes("Logs/"+name,texture.EncodeToPNG());}finally{cam.targetTexture=null;RenderTexture.active=active;rt.Release();UnityEngine.Object.DestroyImmediate(rt);UnityEngine.Object.DestroyImmediate(texture);}}
    public static void ConfigureAndPreview(){Configure();Capture();}
    [MenuItem("Doosan/ROS 2/Show Gas Tank Map")]
    public static void FocusZone(){var source=GameObject.Find("Tank_Outlet_Gas_Source_120ppm");if(!source)return;var plume=GameObject.Find("Gas_Leak_Visualization");Selection.activeGameObject=plume?plume:source;var view=SceneView.lastActiveSceneView??EditorWindow.GetWindow<SceneView>();view.LookAtDirect(source.transform.position,Quaternion.Euler(24,235,0),4.5f);view.Focus();if(!Application.isPlaying&&plume){var ps=plume.GetComponent<ParticleSystem>();ps.Simulate(2,true,true,false);}}
    [Serializable] class Report{public Vector3 source;public float alarmRadius,cautionRadius;public int waypoints;public string providedTank,providedEffect;}
}
