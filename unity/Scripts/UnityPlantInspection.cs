using System;
using UnityEngine;
using Doosan.Ros2;

/// <summary>Scene-based simulated temperature and gas. ROS performs image reading and alarm decisions.</summary>
[RequireComponent(typeof(UnityRosBridge))]
public class UnityPlantInspection : MonoBehaviour
{
    public Transform gasSource;
    public Transform inspectionLookTarget;
    public float inspectionFov=28;
    public float gasPeakPpm=120,gasSigma=1.25f,ambientC=22,sensorHz=5;
    public int thermalWidth=80,thermalHeight=60;
    UnityRosBridge bridge;
    Texture2D heatPreview;
    Color[] colors;
    double nextSample;
    string mission="Waiting for ROS inspection node",lastStatus="";
    double statusReceived=-100;
    InspectionStatus status;
    Quaternion travelCameraRotation;float travelFov;
    [Serializable] public class InspectionStatus{public bool gauge_valid,thermal_valid,gas_valid,thermal_alert,gas_alert,gauge_alert;public float gauge_bar,temperature_c,gas_ppm,exposure_ppm_s;public float[] gauges_bar;}
    void OnEnable(){bridge=GetComponent<UnityRosBridge>();bridge.MessageReceived+=Receive;nextSample=0;if(bridge.sensorCamera){travelCameraRotation=bridge.sensorCamera.transform.localRotation;travelFov=bridge.sensorCamera.fieldOfView;}}
    void OnDisable(){if(bridge)bridge.MessageReceived-=Receive;if(heatPreview)Destroy(heatPreview);}
    void Receive(string json)
    {
        var message=JsonUtility.FromJson<StringEnvelope>(json);
        if(message?.msg==null||message.op!="publish")return;
        if(message.topic=="/mission/status")mission=message.msg.data;
        if(message.topic=="/inspection/status"){
            lastStatus=message.msg.data;status=JsonUtility.FromJson<InspectionStatus>(lastStatus);statusReceived=Time.realtimeSinceStartupAsDouble;
        }
    }
    void Update()
    {
        if(!bridge.sensorCamera||!inspectionLookTarget)return;
        bool inspecting=mission.StartsWith("align:")||mission.StartsWith("inspect:");
        if(inspecting){bridge.sensorCamera.transform.LookAt(inspectionLookTarget.position);bridge.sensorCamera.fieldOfView=inspectionFov;}
        else{bridge.sensorCamera.transform.localRotation=travelCameraRotation;bridge.sensorCamera.fieldOfView=travelFov;}
    }
    void LateUpdate()
    {
        if(!bridge.Connected||!bridge.sensorCamera||Time.timeAsDouble<nextSample)return;
        double now=Time.timeAsDouble;nextSample=now+1/Math.Max(.1,sensorHz);
        int w=Mathf.Clamp(thermalWidth,16,320),h=Mathf.Clamp(thermalHeight,12,240);
        var values=new float[w*h];if(colors==null||colors.Length!=values.Length)colors=new Color[values.Length];
        for(int y=0;y<h;y++)for(int x=0;x<w;x++){
            var ray=bridge.sensorCamera.ViewportPointToRay(new Vector3((x+.5f)/w,1-(y+.5f)/h,0));float c=ambientC;
            if(Physics.Raycast(ray,out var hit,30,bridge.worldLayers,QueryTriggerInteraction.Ignore)){
                var heat=hit.collider.GetComponentInParent<SimulatedHeatSurface>();if(heat)c=heat.temperatureC;
            }
            values[y*w+x]=c;float t=Mathf.Clamp01((c-ambientC)/80);
            colors[(h-1-y)*w+x]=Color.Lerp(new Color(.08f,.02f,.25f),new Color(1,.85f,.1f),t);
        }
        bridge.Publish("/thermal/image",new Doosan.Ros2.Image{header=new Header{stamp=RosTime.FromSeconds(now),frame_id="camera_optical_frame"},width=(uint)w,height=(uint)h,encoding="32FC1",step=(uint)(w*4),data=Convert.ToBase64String(RosCoordinates.PointBytes(values))});
        if(!heatPreview){heatPreview=new Texture2D(w,h,TextureFormat.RGB24,false);heatPreview.filterMode=FilterMode.Point;}
        heatPreview.SetPixels(colors);heatPreview.Apply();
        float ppm=0;if(gasSource){var p=transform.position-gasSource.position;float sigma=Mathf.Max(.1f,gasSigma);ppm=gasPeakPpm*Mathf.Exp(-(p.x*p.x+p.z*p.z)/(2*sigma*sigma));}
        bridge.Publish("/gas/concentration",new Float32Value{data=ppm});
    }
    void OnGUI()
    {
        GUI.color=Color.white;GUI.Box(new Rect(16,90,530,190),GUIContent.none);
        GUI.Label(new Rect(28,96,505,24),"SIMULATED PLANT INSPECTION | ROS 2");
        GUI.Label(new Rect(28,120,505,24),mission);
        bool fresh=status!=null&&Time.realtimeSinceStartupAsDouble-statusReceived<2;
        string gaugeLabel="Gauge: awaiting a valid camera reading";
        if(fresh&&status.gauge_valid){
            gaugeLabel=status.gauges_bar!=null&&status.gauges_bar.Length==3?"G1 / G2 / G3: "+string.Join(" / ",Array.ConvertAll(status.gauges_bar,v=>v.ToString("F1")))+" bar":"Gauge: "+status.gauge_bar.ToString("F2")+" bar";
        }
        GUI.Label(new Rect(28,145,380,24),gaugeLabel);
        GUI.Label(new Rect(28,169,380,24),fresh&&status.thermal_valid?"Thermal: "+status.temperature_c.ToString("F1")+" C"+(status.thermal_alert?"  HOT":"") : "Thermal: no fresh data");
        GUI.Label(new Rect(28,193,380,24),fresh&&status.gas_valid?"Gas: "+status.gas_ppm.ToString("F1")+" ppm | dose "+status.exposure_ppm_s.ToString("F1")+" ppm.s" : "Gas: no fresh data");
        GUI.Label(new Rect(28,220,490,24),fresh&&status.gas_alert?"GAS ALARM: returning along the visited route":"Gauge scale: 0-100 bar | synthetic temperature/gas");
        if(heatPreview)GUI.DrawTexture(new Rect(412,147,120,90),heatPreview,ScaleMode.ScaleToFit);
    }
}
