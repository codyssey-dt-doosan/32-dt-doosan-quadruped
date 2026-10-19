using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public static class DoosanVideoSceneBuilder
{
    const string ScenePath="Assets/Scenes/Plant_VideoReference.unity";
    const string Generated="Assets/DoosanReference";
    [MenuItem("Doosan/Build Video Reference Scene")]
    public static void Build()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new InvalidOperationException("Stop Play before building the layout.");
        Directory.CreateDirectory("Assets/Scenes");Directory.CreateDirectory(Generated);
        if(SceneManager.GetActiveScene().path!="Assets/OutdoorsScene.unity")EditorSceneManager.OpenScene("Assets/OutdoorsScene.unity");
        EditorSceneManager.SaveScene(SceneManager.GetActiveScene(),ScenePath,true);
        EditorSceneManager.OpenScene(ScenePath);
        var scene=SceneManager.GetActiveScene();
        foreach(var go in scene.GetRootGameObjects())
            if(go.name!="Sun"&&go.name!="Sky and Fog Volume"&&go.name!="StaticLightingSky")UnityEngine.Object.DestroyImmediate(go);
        var environment=new GameObject("01_Environment");
        var terrain=new GameObject("02_Corridor_Stairs_Ramp");
        var facility=new GameObject("03_Factory_Equipment");
        var obstacles=new GameObject("04_Obstacles");
        var routes=new GameObject("05_Patrol_Waypoints");
        var cameras=new GameObject("06_Cameras");
        var lighting=new GameObject("07_Interior_Lighting");
        var hall=Spawn("Assets/Map/factory_hall_1.prefab",terrain.transform,Vector3.zero,Quaternion.Euler(-90,0,0));
        var factory=Spawn("Assets/Map/factory_inner_1.prefab",facility.transform,new Vector3(14.953f,0,0),Quaternion.Euler(-90,0,0));
        ReplaceColliders(hall);ReplaceColliders(factory);
        // Match the corridor's open end to a free interval in the factory west entrance.
        Physics.SyncTransforms();
        float doorZ=FindDoor(factory);
        hall.transform.position=new Vector3(0,0,doorZ-1.25f);
        float lane=doorZ-.55f;
        var machine=Spawn("Assets/object/fac_gastank_1.prefab",facility.transform,new Vector3(13,0,1),Quaternion.Euler(0,90,0));
        ReplaceColliders(machine);
        Stack(obstacles.transform,new Vector3(-23,0,doorZ+.9f),3,10);
        Stack(obstacles.transform,new Vector3(-10,0,doorZ-.95f),2,-12);
        Stack(obstacles.transform,new Vector3(4,0,-6.6f),3,0);
        Stack(obstacles.transform,new Vector3(21,0,5.6f),2,15);
        var leak=Spawn("Assets/effect/gas_spurt_1.prefab",facility.transform,new Vector3(8.65f,1.1f,2.1f),Quaternion.Euler(-90,0,0));
        leak.name="Leak_Inspection_Visual_Only";
        var ps=leak.GetComponent<ParticleSystem>();if(ps){var main=ps.main;main.startSize=.12f;main.startLifetime=1.3f;main.startSpeed=.35f;main.maxParticles=100;var emission=ps.emission;emission.rateOverTime=12;}
        var root=new GameObject("Spot_Robot");root.transform.position=new Vector3(-44,Ground(hall,-44,lane)+.045f,lane);root.transform.rotation=Quaternion.Euler(0,90,0);root.layer=9;
        var robot=Spawn("Assets/object/robot_spot_1.prefab",root.transform,Vector3.zero,Quaternion.identity);
        robot.transform.localPosition=Vector3.zero;robot.transform.localRotation=Quaternion.Euler(0,180,0);
        foreach(var old in robot.GetComponentsInChildren<MonoBehaviour>())if(old.GetType().Name=="SpotPatrol")old.enabled=false;
        foreach(var c in robot.GetComponentsInChildren<Collider>())c.enabled=false;
        foreach(var rb in robot.GetComponentsInChildren<Rigidbody>()){rb.isKinematic=true;rb.useGravity=false;}
        SetLayer(robot,9);
        var cc=root.AddComponent<CharacterController>();cc.height=.66f;cc.center=new Vector3(0,.33f,0);cc.radius=.17f;cc.stepOffset=.24f;cc.slopeLimit=45;cc.skinWidth=.025f;
        var drive=root.AddComponent<DoosanReferencePatrol>();drive.robotAnimator=robot.GetComponent<Animator>();drive.robotAnimator.runtimeAnimatorController=AssetDatabase.LoadAssetAtPath<RuntimeAnimatorController>("Assets/object/SpotAnim.controller");drive.speed=1.3f;
        var points=new List<Transform>();
        Point(points,routes.transform,"01_Ramp_Approach",-36,lane);Point(points,routes.transform,"02_Ramp_Top",-30,lane);
        Point(points,routes.transform,"03_Corridor_Clear",-5,doorZ+.4f);
        Point(points,routes.transform,"04_Factory_Entry",.75f,doorZ);
        var nav=new DoosanLayoutNavigation();var previous=new Vector3(.75f,0,doorZ);
        var targets=new[]{new Vector3(6,0,5.2f),new Vector3(8.6f,0,4.1f),new Vector3(19,0,4.7f),new Vector3(21,0,0),new Vector3(20,0,-6.2f),new Vector3(6,0,-5.4f),new Vector3(.75f,0,doorZ)};
        var names=new[]{"North_Aisle","Inspect_Gauges","Machine_East","East_Aisle","South_Aisle","Return_Aisle","Factory_Exit"};
        for(int ti=0;ti<targets.Length;ti++){
            var path=nav.Path(previous,targets[ti]);
            for(int pi=0;pi<path.Count;pi++){var p=path[pi];Point(points,routes.transform,pi==path.Count-1?names[ti]:"Transit_"+ti+"_"+pi,p.x,p.z);}
            previous=path[path.Count-1];
        }
        Point(points,routes.transform,"Corridor_Return",-30,lane);
        Point(points,routes.transform,"Home",-44,lane);drive.route=points.ToArray();
        var mainCam=CameraObject("Main Camera",cameras.transform);mainCam.tag="MainCamera";mainCam.fieldOfView=58;mainCam.nearClipPlane=.04f;mainCam.farClipPlane=150;
        mainCam.gameObject.AddComponent<AudioListener>();drive.followCamera=mainCam;drive.PositionCamera(true);
        var inset=CameraObject("Corridor_Fixed_Camera",cameras.transform);inset.fieldOfView=64;inset.rect=new Rect(.02f,.025f,.24f,.24f);inset.depth=1;inset.nearClipPlane=.05f;inset.farClipPlane=120;
        inset.transform.position=new Vector3(-43,1.5f,doorZ+.9f);inset.transform.LookAt(new Vector3(-29,-.2f,doorZ));drive.corridorCamera=inset;
        // Real-time lights avoid the missing bake / black-floor appearance of the imported scene.
        for(int i=0;i<10;i++){float x=-46+i*5;float ground=x< -32?-.9f:0;Lamp(lighting.transform,"Corridor_Light_"+i,new Vector3(x,ground+2.7f,doorZ),1800,6);}
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)Lamp(lighting.transform,"Factory_Light_"+i+"_"+j,new Vector3(4+i*8,5.8f,-5+j*5),4500,11);
        var sun=GameObject.Find("Sun");if(sun){var light=sun.GetComponent<Light>();light.intensity=20000;}
        var originalVolume=GameObject.Find("Sky and Fog Volume");
        ConfigureLighting();
        // Give both floor materials a readable, non-mirror finish without changing source assets.
        FixFloor(hall);FixFloor(factory);
        foreach(var go in new[]{GameObject.Find("Sun"),originalVolume,GameObject.Find("StaticLightingSky")})if(go)go.transform.SetParent(environment.transform);
        Physics.SyncTransforms();
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene,ScenePath);AssetDatabase.SaveAssets();
        Selection.activeGameObject=root;
        var view=SceneView.lastActiveSceneView;if(view){view.LookAtDirect(new Vector3(-40,.35f,lane),Quaternion.Euler(18,90,0),5);view.Repaint();}
        File.WriteAllText(Path.Combine(Application.temporaryCachePath,"doosan-layout-build.txt"),"Scene="+ScenePath+"\nDoorZ="+doorZ+"\nRobot="+root.transform.position+"\nWaypoints="+points.Count+"\nFactory bounds="+factory.GetComponent<Renderer>().bounds+"\n");
        Debug.Log("Doosan video-reference layout saved: "+ScenePath);
    }
    static GameObject Spawn(string path,Transform parent,Vector3 p,Quaternion q){var go=(GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(path));go.transform.SetParent(parent,false);go.transform.SetPositionAndRotation(p,q);return go;}
    static void SetLayer(GameObject go,int layer){foreach(var t in go.GetComponentsInChildren<Transform>(true))t.gameObject.layer=layer;}
    static void ReplaceColliders(GameObject go){foreach(var c in go.GetComponentsInChildren<Collider>())UnityEngine.Object.DestroyImmediate(c);foreach(var mf in go.GetComponentsInChildren<MeshFilter>()){if(mf.sharedMesh){var c=mf.gameObject.AddComponent<MeshCollider>();c.sharedMesh=mf.sharedMesh;}}SetLayer(go,8);}
    static float Ground(GameObject hall,float x,float z){var mc=hall.GetComponent<MeshCollider>();return mc.Raycast(new Ray(new Vector3(x,1,z),Vector3.down),out var hit,5)?hit.point.y:-.9f;}
    static float FindDoor(GameObject factory){var mc=factory.GetComponent<MeshCollider>();var b=factory.GetComponent<Renderer>().bounds;float best=1.25f,score=float.MaxValue;for(float z=-7.5f;z<7.5f;z+=.1f){bool clear=true;for(float d=-1.2f;d<=1.2f;d+=.3f){if(mc.Raycast(new Ray(new Vector3(b.min.x+1.4f,1.2f,z+d),Vector3.left),out var h,1.8f)){clear=false;break;}}if(clear&&Mathf.Abs(z-1.25f)<score){score=Mathf.Abs(z-1.25f);best=z;}}return best;}
    static void Stack(Transform p,Vector3 position,int count,float angle){var group=new GameObject("Pallet_Stack");group.transform.SetParent(p);for(int i=0;i<count;i++){var go=Spawn("Assets/object/obs_palette_1.prefab",group.transform,position+Vector3.up*(.18f*i),Quaternion.Euler(0,angle,0));ReplaceColliders(go);}}
    static void Point(List<Transform> list,Transform parent,string name,float x,float z){var go=new GameObject(name);go.transform.SetParent(parent);go.transform.position=new Vector3(x,0,z);list.Add(go.transform);}
    static Camera CameraObject(string name,Transform parent){var go=new GameObject(name);go.transform.SetParent(parent);var c=go.AddComponent<Camera>();go.AddComponent<HDAdditionalCameraData>();return c;}
    static void Lamp(Transform parent,string name,Vector3 p,float candela,float range){var go=new GameObject(name);go.transform.SetParent(parent);go.transform.position=p;var l=go.AddComponent<Light>();l.type=LightType.Point;l.lightUnit=LightUnit.Candela;l.intensity=candela;l.range=range;l.color=new Color(1,.93f,.82f);l.shadows=LightShadows.None;go.AddComponent<HDAdditionalLightData>();}
    [MenuItem("Doosan/Save Independent Lighting")]
    public static void ConfigureLighting()
    {
        var volumeObject=GameObject.Find("Sky and Fog Volume");if(!volumeObject)return;
        var volume=volumeObject.GetComponent<Volume>();
        var profile=ScriptableObject.CreateInstance<VolumeProfile>();profile.name="Plant_Video_Lighting";
        foreach(var component in volume.sharedProfile.components)profile.components.Add(UnityEngine.Object.Instantiate(component));
        var exposure=profile.TryGet<Exposure>(out var e)?e:profile.Add<Exposure>();exposure.mode.Override(ExposureMode.Fixed);exposure.fixedExposure.Override(8f);
        if(profile.TryGet<Fog>(out var fog))fog.enabled.Override(false);
        SaveAsset(profile,Generated+"/PlantLighting.asset");volume.sharedProfile=profile;
        var sky=GameObject.Find("StaticLightingSky");if(sky)sky.GetComponent<StaticLightingSky>().profile=AssetDatabase.LoadAssetAtPath<VolumeProfile>("Assets/Settings/SkyandFogSettingsProfile.asset");
        var visual=profile.TryGet<VisualEnvironment>(out var ve)?ve:profile.Add<VisualEnvironment>();visual.skyAmbientMode.Override(SkyAmbientMode.Dynamic);
        foreach(var light in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsSortMode.None)){
            if(light.name.StartsWith("Corridor_Light_")||light.name.StartsWith("Factory_Light_")){
                light.lightUnit=LightUnit.Candela;light.intensity=light.name.StartsWith("Corridor")?1800:4500;
                EditorUtility.SetDirty(light);
            }
        }
        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        if(SceneManager.GetActiveScene().path==ScenePath)EditorSceneManager.SaveScene(SceneManager.GetActiveScene());
        AssetDatabase.SaveAssets();
    }
    static void SaveAsset(UnityEngine.Object o,string path){
        if(AssetDatabase.LoadMainAssetAtPath(path))AssetDatabase.DeleteAsset(path);
        AssetDatabase.CreateAsset(o,path);
        if(o is VolumeProfile vp)foreach(var component in vp.components)AssetDatabase.AddObjectToAsset(component,vp);
        EditorUtility.SetDirty(o);
    }
    static void FixFloor(GameObject go){foreach(var r in go.GetComponentsInChildren<MeshRenderer>()){var mats=r.sharedMaterials;for(int i=0;i<mats.Length;i++)if(mats[i].name.Contains("Floor")||mats[i].name.Contains("Uphill")){var original=mats[i];var m=new Material(Shader.Find("HDRP/Lit"));m.name=original.name+"_Reference";m.SetColor("_BaseColor",new Color(.65f,.65f,.65f));m.SetTexture("_BaseColorMap",original.GetTexture("_BaseColorMap"));m.SetFloat("_Smoothness",.2f);m.SetFloat("_Metallic",0);m.SetFloat("_DoubleSidedEnable",1);HDMaterial.ValidateMaterial(m);SaveAsset(m,Generated+"/"+m.name+".mat");mats[i]=m;}r.sharedMaterials=mats;}}
}
