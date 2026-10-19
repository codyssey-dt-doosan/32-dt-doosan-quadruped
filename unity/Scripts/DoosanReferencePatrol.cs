using System;
using System.IO;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Video reference staging demo. Local animation and collision-aware motion;
/// this is not a ROS controller or a physics-based quadruped locomotion model.</summary>
[RequireComponent(typeof(CharacterController))]
public class DoosanReferencePatrol : MonoBehaviour
{
    public Transform[] route;
    public Camera followCamera;
    public Camera corridorCamera;
    public Animator robotAnimator;
    public float speed = 1.3f;
    public float followDistance = 3.1f;
    public float followHeight = 1.35f;
    public bool automatic = true;
    public bool paused;
    public int routeIndex;
    CharacterController body;
    float vertical, dwell;
    Vector3 initialPosition;
    float startTime;
    bool recorded;
    void Start()
    {
        body=GetComponent<CharacterController>();
        initialPosition=transform.position;startTime=Time.time;
        if(robotAnimator){robotAnimator.applyRootMotion=false;robotAnimator.Play("spot_move",0,0);}
        PositionCamera(true);
    }
    void Update()
    {
        var kb=Keyboard.current;
        if(kb!=null){
            if(kb.spaceKey.wasPressedThisFrame)paused=!paused;
            if(kb.tabKey.wasPressedThisFrame)automatic=!automatic;
            if(kb.vKey.wasPressedThisFrame&&corridorCamera)corridorCamera.enabled=!corridorCamera.enabled;
        }
        Vector3 direction=Vector3.zero;
        if(!paused){
            if(automatic&&route!=null&&route.Length>0){
                if(dwell>0)dwell-=Time.deltaTime;
                else{
                    var delta=route[routeIndex].position-transform.position;delta.y=0;
                    if(delta.magnitude<.10f){dwell=route[routeIndex].name.Contains("Inspect")?2f:.1f;routeIndex=(routeIndex+1)%route.Length;}
                    else direction=delta.normalized;
                }
            }else if(kb!=null){
                float x=(kb.dKey.isPressed||kb.rightArrowKey.isPressed?1:0)-(kb.aKey.isPressed||kb.leftArrowKey.isPressed?1:0);
                float z=(kb.wKey.isPressed||kb.upArrowKey.isPressed?1:0)-(kb.sKey.isPressed||kb.downArrowKey.isPressed?1:0);
                transform.Rotate(0,x*100f*Time.deltaTime,0);
                direction=transform.forward*z;
            }
        }
        if(body.isGrounded&&vertical<0)vertical=-2;
        else vertical-=9.81f*Time.deltaTime;
        if(automatic&&direction.sqrMagnitude>.01f)transform.rotation=Quaternion.Slerp(transform.rotation,Quaternion.LookRotation(direction),8f*Time.deltaTime);
        body.Move((direction*speed+Vector3.up*vertical)*Time.deltaTime);
        if(robotAnimator)robotAnimator.speed=direction.sqrMagnitude>.01f?1:0;
        if(!recorded&&Time.time-startTime>12f){
            recorded=true;
            File.WriteAllText(Path.Combine(Application.dataPath,"../Logs/PlantReferenceRuntime.json"),
              JsonUtility.ToJson(new Observation{elapsed=Time.time-startTime,start=initialPosition,end=transform.position,distance=Vector3.Distance(initialPosition,transform.position),grounded=body.isGrounded,waypoint=routeIndex,animatorPlaying=robotAnimator&&robotAnimator.GetCurrentAnimatorStateInfo(0).IsName("spot_move")},true));
        }
    }
    void LateUpdate(){PositionCamera(false);}
    public void PositionCamera(bool snap)
    {
        if(!followCamera)return;
        Vector3 target=transform.position+Vector3.up*.36f;
        Vector3 desired=transform.position-transform.forward*followDistance+Vector3.up*followHeight;
        Vector3 delta=desired-target;
        if(Physics.SphereCast(target,.12f,delta.normalized,out var hit,delta.magnitude,1<<8,QueryTriggerInteraction.Ignore))desired=target+delta.normalized*Mathf.Max(.5f,hit.distance-.15f);
        followCamera.transform.position=snap?desired:Vector3.Lerp(followCamera.transform.position,desired,1-Mathf.Exp(-8*Time.deltaTime));
        followCamera.transform.LookAt(target);
    }
    [Serializable] class Observation{public float elapsed;public Vector3 start,end;public float distance;public bool grounded;public int waypoint;public bool animatorPlaying;}
    void OnGUI()
    {
        GUI.color=Color.white;
        GUI.Box(new Rect(16,14,450,70),GUIContent.none);
        GUI.Label(new Rect(28,22,430,24),"PLANT PATROL  |  "+(automatic?"AUTO":"MANUAL")+(paused?"  PAUSED":""));
        GUI.Label(new Rect(28,46,430,24),"Space: pause   Tab: auto/manual   WASD: move   V: inset");
    }
    void OnDrawGizmosSelected(){if(route==null)return;Gizmos.color=Color.cyan;for(int i=0;i<route.Length;i++)if(route[i]){Gizmos.DrawSphere(route[i].position,.12f);if(i>0&&route[i-1])Gizmos.DrawLine(route[i-1].position,route[i].position);}}
}
