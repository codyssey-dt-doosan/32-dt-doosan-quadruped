using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Doosan.Ros2
{
    /// <summary>Desktop rosbridge transport. Background tasks never call Unity APIs.
    /// At most one pending sample per topic prevents a slow network from building a sensor backlog.</summary>
    public sealed class RosbridgeTransport : IDisposable
    {
        public struct Received { public string json;public long ticks; }
        readonly Uri uri;
        readonly string[] handshake;
        readonly CancellationTokenSource lifetime=new CancellationTokenSource();
        readonly ConcurrentDictionary<string,string> latest=new ConcurrentDictionary<string,string>();
        readonly ConcurrentQueue<Received> incoming=new ConcurrentQueue<Received>();
        ClientWebSocket socket;
        Task worker;
        volatile bool connected;
        volatile string status="Disconnected";
        int generation;
        public bool Connected=>connected;
        public string Status=>status;
        public int Generation=>Volatile.Read(ref generation);
        public static double Age(long ticks)=>(Stopwatch.GetTimestamp()-ticks)/(double)Stopwatch.Frequency;
        public RosbridgeTransport(string address,string[] registrations){uri=new Uri(address);handshake=registrations;}
        public void Start(){worker=Task.Run(Run);}
        public bool TryReceive(out Received message)=>incoming.TryDequeue(out message);
        public void Publish(string topic,string json){if(connected)latest[topic]=json;}
        async Task Run()
        {
            while(!lifetime.IsCancellationRequested){
                using(var connection=CancellationTokenSource.CreateLinkedTokenSource(lifetime.Token))
                using(var client=new ClientWebSocket()){
                    socket=client;
                    try{
                        status="Connecting";
                        using(var timeout=CancellationTokenSource.CreateLinkedTokenSource(connection.Token)){
                            timeout.CancelAfter(TimeSpan.FromSeconds(5));await client.ConnectAsync(uri,timeout.Token).ConfigureAwait(false);
                        }
                        foreach(var json in handshake)await Send(client,json,connection.Token).ConfigureAwait(false);
                        Interlocked.Increment(ref generation);connected=true;status="Connected";
                        var receive=Receive(client,connection.Token);var send=SendPending(client,connection.Token);
                        await Task.WhenAny(receive,send).ConfigureAwait(false);
                        connection.Cancel();client.Abort();
                        try{await Task.WhenAll(receive,send).ConfigureAwait(false);}catch(OperationCanceledException){}catch(WebSocketException){}
                    }catch(OperationCanceledException){}catch(Exception ex){status="Reconnecting: "+ex.GetType().Name;}
                    finally{
                        connected=false;latest.Clear();while(incoming.TryDequeue(out _)){}socket=null;
                        if(!lifetime.IsCancellationRequested)status="Reconnecting";
                    }
                }
                try{await Task.Delay(1000,lifetime.Token).ConfigureAwait(false);}catch(OperationCanceledException){}
            }
            status="Disconnected";
        }
        async Task Receive(ClientWebSocket client,CancellationToken token)
        {
            var buffer=new byte[8192];
            while(!token.IsCancellationRequested){
                using(var stream=new MemoryStream()){
                    WebSocketReceiveResult result;
                    do{
                        result=await client.ReceiveAsync(new ArraySegment<byte>(buffer),token).ConfigureAwait(false);
                        if(result.MessageType==WebSocketMessageType.Close)return;
                        if(stream.Length+result.Count>65536)throw new InvalidDataException("Oversized command message");
                        stream.Write(buffer,0,result.Count);
                    }while(!result.EndOfMessage);
                    if(result.MessageType!=WebSocketMessageType.Text)continue;
                    while(incoming.Count>=32)incoming.TryDequeue(out _);
                    incoming.Enqueue(new Received{json=Encoding.UTF8.GetString(stream.ToArray()),ticks=Stopwatch.GetTimestamp()});
                }
            }
        }
        async Task SendPending(ClientWebSocket client,CancellationToken token)
        {
            while(!token.IsCancellationRequested){
                if(latest.TryRemove("/clock",out var clock))await Send(client,clock,token).ConfigureAwait(false);
                foreach(var topic in latest.Keys)if(latest.TryRemove(topic,out var json))await Send(client,json,token).ConfigureAwait(false);
                await Task.Delay(5,token).ConfigureAwait(false);
            }
        }
        static async Task Send(ClientWebSocket client,string json,CancellationToken token){var bytes=Encoding.UTF8.GetBytes(json);await client.SendAsync(new ArraySegment<byte>(bytes),WebSocketMessageType.Text,true,token).ConfigureAwait(false);}
        public void Dispose(){connected=false;lifetime.Cancel();socket?.Abort();latest.Clear();}
    }
}
