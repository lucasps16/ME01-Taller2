/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 University of Kansas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Justin Rohrer <rohrej@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/core-module.h"
#include "ns3/opengym-module.h"
#include <cstdio>


using namespace ns3;
using namespace dsr;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");

uint32_t global_PacketsReceived;
float distance_global= 1000.0; 
float distance_change= 0; 
float initialXC1 =0.0;
float initialYC1 =0.0;
float initialXC2 =200.0;
float initialYC2 =500.0;
float initialXC3 =200.0;
float initialYC3 =1000.0;


class RoutingExperiment
{
public:
  RoutingExperiment ();
  void Run (int nSinks, double txp, std::string CSVfileName);
  //static void SetMACParam (ns3::NetDeviceContainer & devices,
  //                                 int slotDistance);
  std::string CommandSetup (int argc, char **argv);
  //uint32_t getPackets() const { return packetsReceived; }


private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);
  void CheckThroughput ();
  

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;
  int m_nSinks;
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility;
  uint32_t m_protocol;
};

Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
  uint32_t nodeNum = 3;
  float low = 0.0;
  float high = 100.0;
  std::vector<uint32_t> shape = {nodeNum,};
  std::string dtype = TypeNameGet<uint32_t> ();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_UNCOND ("MyGetObservationSpace: " << space);
  return space;
}




Ptr<OpenGymSpace> MyGetActionSpace(void)
{
  uint32_t nodeNum = 3;

  Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (nodeNum);
  NS_LOG_UNCOND ("MyGetActionSpace: " << space);
  return space;
}


bool MyGetGameOver(void)
{

  bool isGameOver = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 10) {
      isGameOver = true;
  }
  NS_LOG_UNCOND ("MyGetGameOver: " << isGameOver);
  return isGameOver;
}


Ptr<OpenGymDataContainer> MyGetObservation(void)
{
  //Define the base observation space
  uint32_t nodeNum = 3;

  std::vector<uint32_t> shape = {nodeNum,};
  Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);
  //uint32_t nodeNum2 = NodeList::GetNNodes ();
    for (uint32_t i=0; i<nodeNum; i++)
    {
      Ptr<Node> node = NodeList::GetNode(i);

        //Extract the position from the hierarchy 2 nodes
      Ptr<MobilityModel> cpMob = node->GetObject<MobilityModel>();
      Vector m_position = cpMob->GetPosition();
      box->AddValue(m_position.x);
      
    }
  NS_LOG_UNCOND ("MyGetObservation: " << box);
  return box;
}


float 
MyGetReward(void)
{
  return global_PacketsReceived;
}

string convertToString(char* a, int size)
{
    int i;
    string s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}

bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
  NS_LOG_UNCOND ("MyExecuteActions: " << action);
  
  // Get and format actions
  Ptr<OpenGymBoxContainer<uint32_t> > box = DynamicCast<OpenGymBoxContainer<uint32_t> >(action);
  std::vector<uint32_t> actionVector = box->GetData();

  
  uint32_t nodeNum = NodeList::GetNNodes ();
  //Iterate over nodes and check if the nodes are the ones of the second hierarchy
  distance_change+=100;
  for (uint32_t i=0; i<nodeNum; i++)
  {
      Ptr<Node> node = NodeList::GetNode(i);
      //Set location of the nodes of the second hierarchy
      Ptr<MobilityModel> cpMob = node->GetObject<MobilityModel>();
    
      Vector m_position = cpMob->GetPosition();
      m_position.y = m_position.x-distance_change;
      cpMob->SetPosition(m_position);
      
    
  }

  return true;
}



void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGym)
{
  Simulator::Schedule (Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGym);
  openGym->NotifyCurrentState();
}


RoutingExperiment::RoutingExperiment ()
  : port (9),
    bytesTotal (0),
    packetsReceived (0),
    m_CSVfileName ("manet-simulation.output.csv"),
    m_traceMobility (false),
    m_protocol (2) // AODV
{
}

static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
      //std::cout<<"RECIBI UN PAQUETe"<<std::endl;
    }
  else
    {
      oss << " received one packet!";
    }

  
  return oss.str ();
}

void
RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      global_PacketsReceived+=1;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

void
RoutingExperiment::CheckThroughput ()
{
  double kbs = (bytesTotal * 8.0) / 1000;
  bytesTotal = 0;

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ""
      << std::endl;

  out.close ();
  packetsReceived = 0;
  Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CheckThroughput, this);
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return sink;
}

std::string
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd (__FILE__);
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse (argc, argv);
  return m_CSVfileName;
}

int
main (int argc, char *argv[])
{
  RoutingExperiment experiment;
  std::string CSVfileName = experiment.CommandSetup (argc,argv);

  //blank out the last output file and write the column headers
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower" <<
  std::endl;
  out.close ();

  int nSinks = 10;
  double txp = 7.5;

  experiment.Run (nSinks, txp, CSVfileName);

  




}

void
RoutingExperiment::Run (int nSinks, double txp, std::string CSVfileName)
{
  Packet::EnablePrinting ();
  //m_nSinks = nSinks;
  m_txp = txp;
  m_CSVfileName = CSVfileName;


  // m
  //uint32_t packetSize = 1000; // bytes
  //uint32_t numPackets = 100;
  uint32_t n = 10;  // by default, 5x2
  //uint32_t sinkNode = 0; // objective node of cluster 1
  //uint32_t sourceNode = 3; //source node  of cluster 3
  //double envStepTime = 0.6; //seconds, ns3gym env step time interval
  //double interval = 0.2; // seconds


  int nWifis = 50;

  double TotalTime = 200.0;
  std::string rate ("2048bps");
  std::string phyMode ("DsssRate11Mbps");
  std::string tr_name ("manet-routing-compare");
  int nodeSpeed = 20; //in m/s
  int nodePause = 0; //in s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

 // CREATE NODES AND CLUSTERS
  NodeContainer head_clusters;
  head_clusters.Create (3,0);

  NodeContainer Cluster1;
  Cluster1.Add (head_clusters.Get(0));
  Cluster1.Create(n,1);

  NodeContainer Cluster2;
  Cluster2.Add (head_clusters.Get(1));
  Cluster2.Create(n,2);

  NodeContainer Cluster3;
  Cluster3.Add (head_clusters.Get(2));
  Cluster3.Create(n,3);
  
  NodeContainer listClusters [] = {Cluster1,Cluster2,Cluster3,head_clusters};


  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  // NETWORK FOR EACH CLUSTER 
  NetDeviceContainer devicesCluster1 = wifi.Install (wifiPhy, wifiMac, Cluster1);
  NetDeviceContainer devicesCluster2 = wifi.Install (wifiPhy, wifiMac, Cluster2);
  NetDeviceContainer devicesCluster3 = wifi.Install (wifiPhy, wifiMac, Cluster3);
  NetDeviceContainer devicesHC = wifi.Install (wifiPhy, wifiMac, head_clusters);


 // MOBILITY
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";


  // MOBILITY FOR CLUSTER 1
  MobilityHelper mobilityCluster1;
  int64_t streamIndex1 = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos1;
  
  pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
  pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));

  Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();
  streamIndex1 += taPositionAlloc1->AssignStreams (streamIndex1);


    mobilityCluster1.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc1));
  mobilityCluster1.SetPositionAllocator (taPositionAlloc1);
  mobilityCluster1.Install (Cluster1);

  streamIndex1 += mobilityCluster1.AssignStreams (Cluster1, streamIndex1);
  NS_UNUSED (streamIndex1); // From this point, streamIndex is unused

  // MOBILITY FOR CLUSTER 2

  MobilityHelper mobilityCluster2;
  int64_t streamIndex2 = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos2;
  char distanceY2[700]={'\0'};
  pos2.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  sprintf(distanceY2, "ns3::UniformRandomVariable[Min=%f|Max=%f]", 500+distance_global,1000+distance_global);
  pos2.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=101.0|Max=200.0]"));
  pos2.Set ("Y", StringValue (convertToString(distanceY2,700)));

  Ptr<PositionAllocator> taPositionAlloc2 = pos2.Create ()->GetObject<PositionAllocator> ();
  streamIndex2 += taPositionAlloc2->AssignStreams (streamIndex2);

  mobilityCluster2.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc2));
  mobilityCluster2.SetPositionAllocator (taPositionAlloc2);
  mobilityCluster2.Install (Cluster2);

  streamIndex2 += mobilityCluster2.AssignStreams (Cluster2, streamIndex2);
  NS_UNUSED (streamIndex2); // From this point, streamIndex is unused

  // MOBILITY FOR CLUSTER 3

  MobilityHelper mobilityCluster3;
  int64_t streamIndex3 = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos3;
  pos3.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  
  
  char distanceY3[700]={'\0'}; 
  sprintf(distanceY3, "ns3::UniformRandomVariable[Min=%f|Max=%f]", 1000+distance_global,1500+distance_global);
  pos3.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=201.0|Max=300.0]"));
  pos3.Set ("Y", StringValue (convertToString(distanceY3,700)));

  Ptr<PositionAllocator> taPositionAlloc3 = pos3.Create ()->GetObject<PositionAllocator> ();
  streamIndex3 += taPositionAlloc3->AssignStreams (streamIndex3);

    mobilityCluster3.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc3));
  mobilityCluster3.SetPositionAllocator (taPositionAlloc3);
  mobilityCluster3.Install (Cluster3);

  streamIndex3 += mobilityCluster3.AssignStreams (Cluster3, streamIndex3);
  NS_UNUSED (streamIndex3); // From this point, streamIndex is unused

  // MOBILITY FOR CLUSTER HEADS

  MobilityHelper mobilityCH;
  int64_t streamIndex4 = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos4;
  pos4.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos4.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos4.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

  Ptr<PositionAllocator> taPositionAlloc4 = pos4.Create ()->GetObject<PositionAllocator> ();
  streamIndex4 += taPositionAlloc4->AssignStreams (streamIndex4);

    mobilityCH.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc4));
  mobilityCH.SetPositionAllocator (taPositionAlloc4);
  mobilityCH.Install (head_clusters);

  streamIndex4 += mobilityCH.AssignStreams (head_clusters, streamIndex4);
  NS_UNUSED (streamIndex4); // From this point, streamIndex is unused

  
  mobilityCluster1.Install(Cluster1);
  mobilityCluster2.Install(Cluster2);
  mobilityCluster3.Install(Cluster3);
  mobilityCH.Install(head_clusters);

  AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  list.Add (staticRouting,0);
  list.Add (aodv,30);

  internet.SetRoutingHelper (list);
  internet.Install (Cluster1);
  internet.Install (Cluster2);
  internet.Install (Cluster3);
  

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interface1= addressAdhoc.Assign(devicesCluster1);
  Ipv4InterfaceContainer interface2= addressAdhoc.Assign(devicesCluster2);
  Ipv4InterfaceContainer interface3= addressAdhoc.Assign(devicesCluster3);
  Ipv4InterfaceContainer interface4= addressAdhoc.Assign(devicesHC);
  
  Ipv4InterfaceContainer interfaceList [] = {interface1,interface2,interface3,interface4};



  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));


  Ptr<Socket> sink = SetupPacketReceive (interface1.GetAddress(0), Cluster1.Get (0));

  AddressValue remoteAddress (InetSocketAddress (interface1.GetAddress (0), port));
  onoff1.SetAttribute ("Remote", remoteAddress);


  ApplicationContainer temp = onoff1.Install (Cluster3.Get (0));
  

  
  //temp.Start (Seconds (var->GetValue (100.0,101.0)));
  //temp.Stop (Seconds (TotalTime));
  

  std::stringstream ss;
  ss << nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << nodeSpeed;
  std::string sNodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();

  //NS_LOG_INFO ("Configure Tracing.");
  //tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

  //AsciiTraceHelper ascii;
  //Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (tr_name + ".tr").c_str());
  //wifiPhy.EnableAsciiAll (osw);
  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  //Ptr<FlowMonitor> flowmon;
  //FlowMonitorHelper flowmonHelper;
  //flowmon = flowmonHelper.InstallAll ();
  double envStepTime = 0.1; //seconds, ns3gym env step time interval
  uint32_t openGymPort = 5555;
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (openGymPort);
  openGym->SetGetActionSpaceCb( MakeCallback (&MyGetActionSpace) );
  openGym->SetGetObservationSpaceCb( MakeCallback (&MyGetObservationSpace) );
  openGym->SetGetGameOverCb( MakeCallback (&MyGetGameOver) );
  openGym->SetGetObservationCb( MakeCallback (&MyGetObservation) );
  
  openGym->SetGetRewardCb( MakeCallback (&MyGetReward) );
  openGym->SetExecuteActionsCb( MakeCallback (&MyExecuteActions) );
  Simulator::Schedule (Seconds(0.0), &ScheduleNextStateRead, envStepTime, openGym);


  NS_LOG_INFO ("Run Simulation.");

  CheckThroughput ();

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();

  //flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);
  

  Simulator::Destroy ();
}

