#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/buildings-module.h"
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"

#include "LteRoutingExtension.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FirstTryAmbulanceNetwork");

std::ofstream outputFile;

void AddRoutingProtocolToNode(Ptr<Node> node, Ipv4RoutingHelper &routingHelper)
{
    const Ipv4RoutingHelper *routing = routingHelper.Copy();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ptr<Ipv4RoutingProtocol> ipv4RoutingProtocol = routing->Create(node);
    ipv4->SetRoutingProtocol(ipv4RoutingProtocol);

    for(uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
        ipv4RoutingProtocol->NotifyInterfaceUp(i);
}

//print RSRP and RSRQ values to file
void printLteInfo(Ptr<NetDevice> netDeviceA, Ptr<NetDevice> netDeviceB)
{
    Ptr<LteUeNetDevice> deviceA = netDeviceA->GetObject<LteUeNetDevice>();
    Ptr<LteUeNetDevice> deviceB = netDeviceB->GetObject<LteUeNetDevice>();

    outputFile << Simulator::Now().GetSeconds() << "," << deviceA->GetRrc()->GetRsrp() << "," 
                                    << deviceA->GetRrc()->GetRsrq() << "," 
                                    << deviceB->GetRrc()->GetRsrp() << "," 
                                    << deviceB->GetRrc()->GetRsrq() << "\n";

    Simulator::Schedule(Seconds(0.1), &printLteInfo, netDeviceA, netDeviceB);
}

int main(int argc, char *argv[])
{
    outputFile.open("signalStrength.csv");
    outputFile << ",LTE A,,LTE B\n";
    outputFile << "Time,RSRP,RSRQ,RSRP,RSRQ\n";

    int16_t timeIndex = 0;
    int16_t scenario = 0;

    CommandLine cmd (__FILE__);
    cmd.AddValue("timeIndex", "", timeIndex);
    cmd.AddValue("scenario", "", scenario);
    cmd.Parse(argc, argv);

    //_______________________Nodes__________________________________________________

    std::vector<uint16_t> eNBNodesACount = {3,3,2};
    std::vector<uint16_t> eNBNodesBCount = {3,2,2};
    uint16_t otherNodesCount = 2;

    NodeContainer allNodes;
    allNodes.Create(eNBNodesACount[scenario] + eNBNodesBCount[scenario] + otherNodesCount);             //maybe additional nodes needed for lte/5g network
    
    NodeContainer routerNodes = NodeContainer(allNodes.Get(0), allNodes.Get(1));
    NodeContainer eNBNodesA{};
    NodeContainer eNBNodesB{};

    for(int16_t i = 0; i < eNBNodesACount[scenario]; ++i)
        eNBNodesA.Add(allNodes.Get(i + otherNodesCount));
    for(int16_t i = 0; i < eNBNodesBCount[scenario]; ++i)
        eNBNodesB.Add(allNodes.Get(i + otherNodesCount + eNBNodesACount[scenario]));

    //_______________________Mobility_______________________________________________

    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(eNBNodesA);
    mobility.Install(eNBNodesB);
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(routerNodes);
    
    BuildingsHelper::Install(allNodes);

    //_______________________Positioning____________________________________________

    //eNBs
    std::vector<Vector> eNBPositionsA1;
    eNBPositionsA1.push_back(Vector(0, -1000, 0.1));
    eNBPositionsA1.push_back(Vector(200, 300, 50));
    eNBPositionsA1.push_back(Vector(200, 800, 50));

    std::vector<Vector> eNBPositionsB1;
    eNBPositionsB1.push_back(Vector(0, -1000, 0.1));
    eNBPositionsB1.push_back(Vector(400, 1000, 50));
    eNBPositionsB1.push_back(Vector(400, 1500, 50));

    std::vector<Vector> eNBPositionsA2;
    eNBPositionsA2.push_back(Vector(0, -1000, 0.1));
    eNBPositionsA2.push_back(Vector(200, 400, 50));
    eNBPositionsA2.push_back(Vector(200, 1400, 50));

    std::vector<Vector> eNBPositionsB2;
    eNBPositionsB2.push_back(Vector(0, -1000, 0.1));
    eNBPositionsB2.push_back(Vector(400, 900, 50));

    std::vector<Vector> eNBPositionsA3;
    eNBPositionsA3.push_back(Vector(0, -1000, 0.1));
    eNBPositionsA3.push_back(Vector(200, 400, 50));

    std::vector<Vector> eNBPositionsB3;
    eNBPositionsB3.push_back(Vector(0, -1000, 0.1));
    eNBPositionsB3.push_back(Vector(400, 1400, 50));

    std::vector<std::vector<Vector>> scenarioPositionsA;
    scenarioPositionsA.push_back(eNBPositionsA1);
    scenarioPositionsA.push_back(eNBPositionsA2);
    scenarioPositionsA.push_back(eNBPositionsA3);

    std::vector<std::vector<Vector>> scenarioPositionsB;
    scenarioPositionsB.push_back(eNBPositionsB1);
    scenarioPositionsB.push_back(eNBPositionsB2);
    scenarioPositionsB.push_back(eNBPositionsB3);

    for(uint32_t i = 0; i < eNBNodesA.GetN(); ++i)
    {
        Ptr<ConstantPositionMobilityModel> eNBAMobility = eNBNodesA.Get(i)->GetObject<ConstantPositionMobilityModel>();
        eNBAMobility->SetPosition(scenarioPositionsA[scenario][i]);
    }

    for(uint32_t i = 0; i < eNBNodesB.GetN(); ++i)
    {
        Ptr<ConstantPositionMobilityModel> eNBBMobility = eNBNodesB.Get(i)->GetObject<ConstantPositionMobilityModel>();
        eNBBMobility->SetPosition(scenarioPositionsB[scenario][i]);
    }

    //Router
    Ptr<WaypointMobilityModel> ambulanceMobilityModel = routerNodes.Get(0)->GetObject<WaypointMobilityModel>();
    Ptr<WaypointMobilityModel> drMobilityModel = routerNodes.Get(1)->GetObject<WaypointMobilityModel>();
    ambulanceMobilityModel->SetPosition(Vector(300.0, 200.0, 0.1));
    drMobilityModel->SetPosition(Vector(300.0, 200.0, 0.1));

    //_______________________Waypoints______________________________________________

    //Ambulance Router
    std::vector<double> ambulanceEndTimes;
    ambulanceEndTimes.push_back(28.1);      //180km/h
    ambulanceEndTimes.push_back(46.7);      //100km/h
    ambulanceEndTimes.push_back(100.1);     //50km/h
    ambulanceEndTimes.push_back(233.3);     //20km/h
    ambulanceEndTimes.push_back(933.3);     //5km/h

    Waypoint ambulanceWaypointD1(Seconds(0.1), Vector(300, 200, 0.1));
    Waypoint ambulanceWaypointD2(Seconds(ambulanceEndTimes[timeIndex]), Vector(300, 1600, 0.1));
    ambulanceMobilityModel->AddWaypoint(ambulanceWaypointD1);
    ambulanceMobilityModel->AddWaypoint(ambulanceWaypointD2);

    //Dr Router
    std::vector<double> drStartTimes;
    drStartTimes.push_back(29.1);     //180km/h
    drStartTimes.push_back(47.6);     //100km/h
    drStartTimes.push_back(101.1);    //50km/h
    drStartTimes.push_back(234.3);    //20km/h
    drStartTimes.push_back(934.3);    //5km/h

    std::vector<double> drEndTimes;
    drEndTimes.push_back(57.1);     //180km/h
    drEndTimes.push_back(94.2);     //100km/h
    drEndTimes.push_back(201.1);    //50km/h
    drEndTimes.push_back(467.6);    //20km/h
    drEndTimes.push_back(1867.6);    //5km/h

    Waypoint drWaypointD1(Seconds(drStartTimes[timeIndex]), Vector(300, 200, 0.1));
    Waypoint drWaypointD2(Seconds(drEndTimes[timeIndex]), Vector(300, 1600, 0.1));

    drMobilityModel->AddWaypoint(drWaypointD1);
    drMobilityModel->AddWaypoint(drWaypointD2);

    //_______________________LTE____________________________________________________

    //Provider A
    Ptr<LteHelper> lteHelperA = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelperA = CreateObject<PointToPointEpcHelper>(7);
    lteHelperA->SetEpcHelper(epcHelperA);
    lteHelperA->SetAttribute("Scheduler", StringValue("ns3::PfFfMacScheduler"));
    lteHelperA->SetAttribute("PathlossModel", StringValue("ns3::HybridBuildingsPropagationLossModel"));
    lteHelperA->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelperA->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelperA->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(15));
    lteHelperA->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(15));

    NetDeviceContainer eNBDevicesA;
    eNBDevicesA = lteHelperA->InstallEnbDevice(eNBNodesA);
    NetDeviceContainer ueDevicesA;
    ueDevicesA = lteHelperA->InstallUeDevice(routerNodes);
    Ptr<Node> pgwA = epcHelperA->GetPgwNode();

    //Provider B
    Ptr<LteHelper> lteHelperB = CreateObject<LteHelper>(500);
    Ptr<PointToPointEpcHelper> epcHelperB = CreateObject<PointToPointEpcHelper>(13);
    lteHelperB->SetEpcHelper(epcHelperB);
    lteHelperB->SetAttribute("Scheduler", StringValue("ns3::PfFfMacScheduler"));
    lteHelperB->SetAttribute("PathlossModel", StringValue("ns3::HybridBuildingsPropagationLossModel"));
    lteHelperB->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelperB->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelperB->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(25));
    lteHelperB->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(25));

    NetDeviceContainer eNBDevicesB;
    eNBDevicesB = lteHelperB->InstallEnbDevice(eNBNodesB);
    NetDeviceContainer ueDevicesB;
    ueDevicesB = lteHelperB->InstallUeDevice(routerNodes);
    Ptr<Node> pgwB = epcHelperB->GetPgwNode();

    //connection between ProviderA and ProviderB
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2p.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer pgwDevices = p2p.Install(pgwA, pgwB);

    //_______________________5G Mobile Network_____________________________________________________

    //TODO: Module not available substitute????

    //_______________________Protocol Stacks________________________________________

    //Internet Stack
    AodvHelper routingHelper;
    InternetStackHelper stack;
    stack.SetRoutingHelper(routingHelper);
    stack.Install(routerNodes);

    //_______________________Addresses______________________________________________

    //Ip Addresses
    Ipv4InterfaceContainer lteInterfacesA = epcHelperA->AssignUeIpv4Address(NetDeviceContainer(ueDevicesA));    //Base: "7.0.0.0", "255.0.0.0"
    Ipv4InterfaceContainer lteInterfacesB = epcHelperB->AssignUeIpv4Address(NetDeviceContainer(ueDevicesB));    //Base: "13.0.0.0", "255.0.0.0"
    Ipv4AddressHelper ipv4Address;
    ipv4Address.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwInterfaces = ipv4Address.Assign(pgwDevices);

    //attach UEs to an eNB -> Home eNB???
    lteHelperA->Attach(ueDevicesA);
    lteHelperB->Attach(ueDevicesB);

    //_______________________Application Declaration________________________________

    uint port = 10;

    std::string traceFilename = "../workspace/ambulance/traceFHD.txt";
    UdpTraceClientHelper client(lteInterfacesA.GetAddress(1), port, traceFilename);

    ApplicationContainer clientApps = client.Install(routerNodes.Get(0));
    clientApps.Start(Seconds(1.1));
    clientApps.Stop(Seconds(drEndTimes[timeIndex]-1));

    //dr
    UdpServerHelper server(port);
    ApplicationContainer serverApps = server.Install(routerNodes.Get(1));
    serverApps.Start(Seconds(0.1));
    serverApps.Stop(Seconds(drEndTimes[timeIndex]-0.5));

    //_______________________Routing________________________________________________

    //add routing to eNBNodes
    for(uint32_t i = 0; i < eNBNodesA.GetN(); ++i)
        AddRoutingProtocolToNode(eNBNodesA.Get(i), routingHelper);
    for(uint32_t i = 0; i < eNBNodesB.GetN(); ++i)
        AddRoutingProtocolToNode(eNBNodesB.Get(i), routingHelper);

    //add routing to pgw/sgw/mme node of ProviderA and providerB
    AddRoutingProtocolToNode(pgwA, routingHelper);
    AddRoutingProtocolToNode(pgwB, routingHelper);

    Ptr<Node> sgwA = epcHelperA->GetSgwNode();
    Ptr<Node> sgwB = epcHelperB->GetSgwNode();
    AddRoutingProtocolToNode(sgwA, routingHelper);
    AddRoutingProtocolToNode(sgwB, routingHelper);

    Ptr<Node> mmeA = epcHelperA->GetMmeNode();
    Ptr<Node> mmeB = epcHelperB->GetMmeNode();
    AddRoutingProtocolToNode(mmeA, routingHelper);
    AddRoutingProtocolToNode(mmeB, routingHelper);

    //add routing extension to router
    LteRoutingExtension extension1;
    extension1.Install(routerNodes.Get(0));
    extension1.AddDevice(ueDevicesA.Get(0));
    extension1.AddDevice(ueDevicesB.Get(0));
    extension1.SetInterval(0.1);
    extension1.Start();

    LteRoutingExtension extension2;
    extension2.Install(routerNodes.Get(1));
    extension2.AddDevice(ueDevicesA.Get(1));
    extension2.AddDevice(ueDevicesB.Get(1));
    extension2.SetInterval(0.1);
    extension2.Start();


    Simulator::Schedule(Seconds(0.1), &printLteInfo, ueDevicesA.Get(1), ueDevicesB.Get(1));

    //Print routing tables
    //Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> (&std::cout);
    //routingHelper.PrintRoutingTableAllAt(Seconds(19.0), routingStream);
    //aodvRoutingHelper.PrintRoutingTableAt(Seconds(19.0), routerNodes.Get(1), routingStream);
    //aodvRoutingHelper.PrintRoutingTableAt(Seconds(19.0), pgwA, routingStream);
    //aodvRoutingHelper.PrintRoutingTableAt(Seconds(19.0), pgwB, routingStream);
    //routingHelper.PrintRoutingTableAt(Seconds(19.0), eNBNodesA.Get(0), routingStream);

    //_______________________NetAnim________________________________________________

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(pgwA);
    mobility.Install(pgwB);
    mobility.Install(sgwA);
    mobility.Install(sgwB);
    mobility.Install(mmeA);
    mobility.Install(mmeB);

    AnimationInterface anim ("wireless-animation.xml"); // Mandatory

    //optional
    anim.UpdateNodeDescription (routerNodes.Get (0), "Ambulance");
    anim.UpdateNodeDescription (routerNodes.Get (1), "DR");
    anim.UpdateNodeColor (routerNodes.Get (1), 255, 255, 0);

    for (uint32_t i = 0; i < eNBNodesA.GetN(); ++i)
    {
        anim.UpdateNodeDescription (eNBNodesA.Get (i), "eNBA");
        anim.UpdateNodeColor (eNBNodesA.Get (i), 0, 0, 255);
    }
    for (uint32_t i = 0; i < eNBNodesB.GetN(); ++i)
    {
        anim.UpdateNodeDescription (eNBNodesB.Get (i), "eNBB");
        anim.UpdateNodeColor (eNBNodesB.Get (i), 0, 255, 0);
    }

    anim.EnablePacketMetadata ();
    anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (5), Seconds (0.25));
    //anim.EnableWifiMacCounters (Seconds (0), Seconds (10));
    //anim.EnableWifiPhyCounters (Seconds (0), Seconds (10));

    //_______________________Simulation_____________________________________________
    
    Simulator::Stop(Seconds(drEndTimes[timeIndex]));
    Simulator::Run();
    Simulator::Destroy();
    outputFile.close();
    return 0;
}

/*
    NodeContainer endDeviceNodes = NodeContainer(allNodes.Get(2), allNodes.Get(3), allNodes.Get(4), allNodes.Get(5), allNodes.Get(6));
    NodeContainer ambulanceApNodes = NodeContainer(routerNodes.Get(0));
    NodeContainer ambulanceStaNodes = NodeContainer(endDeviceNodes.Get(0), endDeviceNodes.Get(1), endDeviceNodes.Get(2), endDeviceNodes.Get(3));
    NodeContainer drApNodes = NodeContainer(routerNodes.Get(1));
    NodeContainer drStaNodes = NodeContainer(endDeviceNodes.Get(4));

    //_______________________Buildings______________________________________________

    double xMin = 0.0;
    double xMax = 20.0;
    double yMin = 450.0;
    double yMax = 500.0;
    double zMin = 0.0;
    double zMax = 20;

    Ptr<Building> building = CreateObject<Building>();
    building->SetBoundaries(Box(xMin, xMax, yMin, yMax, zMin, zMax));
    building->SetBuildingType(Building::Residential);
    building->SetExtWallsType(Building::ConcreteWithWindows);
    building->SetNFloors(9);
    building->SetNRoomsX(2);
    building->SetNRoomsY(5);

    Rectangle boundingBox(0, 1000, 0, 500);
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue(boundingBox));
    mobility.Install(endDeviceNodes);
    mobility.Install(ambulanceStaNodes.Get(1));
    mobility.Install(ambulanceStaNodes.Get(2));
    mobility.Install(drStaNodes);

    //End Devices
    Ptr<RandomWalk2dMobilityModel> phone = endDeviceNodes.Get(0)->GetObject<RandomWalk2dMobilityModel>();
    Ptr<RandomWalk2dMobilityModel> bodyCam = endDeviceNodes.Get(1)->GetObject<RandomWalk2dMobilityModel>();
    Ptr<RandomWalk2dMobilityModel> medicalDevice = endDeviceNodes.Get(2)->GetObject<RandomWalk2dMobilityModel>();
    Ptr<RandomWalk2dMobilityModel> panoramaCam = endDeviceNodes.Get(3)->GetObject<RandomWalk2dMobilityModel>();
    Ptr<RandomWalk2dMobilityModel> vrHeadset = endDeviceNodes.Get(4)->GetObject<RandomWalk2dMobilityModel>();
    phone->SetPosition(Vector(25.0, 471.0, 0.1));
    bodyCam->SetPosition(Vector(26.0, 470.0, 0.1));
    medicalDevice->SetPosition(Vector(24.0, 470.0, 0.1));
    panoramaCam->SetPosition(Vector(25.0, 470.0, 1.0));
    vrHeadset->SetPosition(Vector(1000.0, 4.0, 0.1));

    //_______________________Wifi___________________________________________________
    //Wifi Setup
    
    //ambulance Wifi
    YansWifiChannelHelper ambulanceWifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper ambulanceWifiPhy;
    ambulanceWifiPhy.SetChannel(ambulanceWifiChannel.Create());
    WifiHelper ambulanceWifi;
    ambulanceWifi.SetRemoteStationManager("ns3::AarfWifiManager");
    WifiMacHelper ambulanceWifiMac;
    Ssid ambulanceSsid = Ssid("ns-3-ssid");
    ambulanceWifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ambulanceSsid), "ActiveProbing", BooleanValue(false));
    NetDeviceContainer ambulanceStaDevices = ambulanceWifi.Install(ambulanceWifiPhy, ambulanceWifiMac, ambulanceStaNodes);
    ambulanceWifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ambulanceSsid));
    NetDeviceContainer ambulanceApDevices = ambulanceWifi.Install(ambulanceWifiPhy, ambulanceWifiMac, ambulanceApNodes);

    //Dr Wifi
    YansWifiChannelHelper drWifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper drWifiPhy;
    drWifiPhy.SetChannel(drWifiChannel.Create());
    WifiHelper drWifi;
    drWifi.SetRemoteStationManager("ns3::AarfWifiManager");
    WifiMacHelper drWifiMac;
    Ssid drSsid = Ssid("ns-3-ssid");
    drWifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(drSsid), "ActiveProbing", BooleanValue(false));
    NetDeviceContainer drStaDevices = drWifi.Install(drWifiPhy, drWifiMac, drStaNodes);
    drWifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(drSsid));
    NetDeviceContainer drApDevices = drWifi.Install(drWifiPhy, drWifiMac, drApNodes);

    ipv4Address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ambulanceApInterfaces = ipv4Address.Assign(ambulanceApDevices);
    Ipv4InterfaceContainer ambulanceStaInterfaces = ipv4Address.Assign(ambulanceStaDevices);
    ipv4Address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer drApInterfaces = ipv4Address.Assign(drApDevices);
    Ipv4InterfaceContainer drStaInterfaces = ipv4Address.Assign(drStaDevices);

    uint port = 9;
    UdpEchoServerHelper server(port);
    ApplicationContainer serverApps = server.Install(routerNodes.Get(1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpEchoClientHelper client(lteInterfacesA.GetAddress(1), port);
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = client.Install(routerNodes.Get(0));
    clientApps.Start(Seconds(6.0));
    clientApps.Stop(Seconds(10.0));

    //_______________________Tracing/Other Output___________________________________

    //AsciiTraceHelper tracer;
    //p2p.EnableAsciiAll(tracer.CreateFileStream("point2point.tr"));
    //ambulanceWifiPhy.EnableAsciiAll(tracer.CreateFileStream("wifi.tr"));
    //drWifiPhy.EnableAsciiAll(tracer.CreateFileStream("wifi.tr"));
    //p2p.EnablePcapAll("point2point");
    //ambulanceWifiPhy.EnablePcap("wifi", ambulanceApDevices.Get(0));
    //drWifiPhy.EnablePcap("wifi", drApDevices.Get(0));
    lteHelperA->EnableTraces();
    lteHelperB->EnableTraces();
    //p2p.EnablePcap("point2point", pgwB->GetId(), 0);

    //get sta positions
    std::ostringstream oss;
    oss << "/NodeList/" << endDeviceNodes.Get(0)->GetId() << "/$ns3::MobilityModel/CourseChange";
    Config::Connect(oss.str(), MakeCallback(&CourseChange));

    //std::cout << ambulanceApDevices.GetN() << std::endl;
*/