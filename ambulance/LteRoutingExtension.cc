#include "LteRoutingExtension.h"

LteRoutingExtension::LteRoutingExtension()
{


}
LteRoutingExtension::~LteRoutingExtension()
{

}

void LteRoutingExtension::Install(Ptr<Node> pNode)
{
    node = pNode;
    ipv4 = node->GetObject<Ipv4>();
}

void LteRoutingExtension::Start()
{
    currentlyUsedDeviceIndex = 0;
    currentlyUsedDevice = devices[0];
    ActivateRoutingOver(0);

    Simulator::Schedule(Seconds(1 + interval), &LteRoutingExtension::CheckSignalStrength, this);
}

void LteRoutingExtension::AddDevice(Ptr<NetDevice> netDevice)
{
    Ptr<LteUeNetDevice> device = netDevice->GetObject<LteUeNetDevice>();
    devices.push_back(device);
    interfaceStatuses.push_back(InterfaceStatus::UP);
    lastWeightedAverage.push_back(0);
    lastTrendValue.push_back(0);
    DeactivateRoutingOver(devices.size()-1);
}

void LteRoutingExtension::SetInterval(double pInterval)
{
    interval = pInterval;
}

void LteRoutingExtension::CheckSignalStrength()
{
    PreSendRouteRequest();
    uint32_t bestDeviceIndex = GetBestSignalDeviceIndex();

    if(bestDeviceIndex != currentlyUsedDeviceIndex)
    {
        currentlyUsedDevice = devices[bestDeviceIndex];
        currentlyUsedDeviceIndex = bestDeviceIndex;
        NS_LOG_UNCOND("changing network at " << Simulator::Now().GetSeconds());

        if(interfaceStatuses[bestDeviceIndex] == InterfaceStatus::DOWN)
            ActivateRoutingOver(bestDeviceIndex);

        for(uint32_t i = 0; i < devices.size(); ++i)
        {
            if(i != bestDeviceIndex && interfaceStatuses[i] == InterfaceStatus::UP)
                DeactivateRoutingOver(i);
        }
    }
    
    Simulator::Schedule(Seconds(interval), &LteRoutingExtension::CheckSignalStrength, this);
}

void LteRoutingExtension::PreSendRouteRequest()
{
    uint32_t bestDeviceIndex = 0;
    double bestRsrp = 0;

    for(uint32_t i = 0; i < devices.size(); ++i)
    {
        if(i == currentlyUsedDeviceIndex)
            continue;

        double currentRsrp = PredictSignalStrength(i);
        
        if(currentRsrp > bestRsrp)
        {
            bestDeviceIndex = i;
            bestRsrp = currentRsrp;
        }
    }

    if(bestRsrp > PredictSignalStrength(currentlyUsedDeviceIndex))
    {
        Ptr<LteUeNetDevice> device = devices[bestDeviceIndex];
        uint32_t interface = ipv4->GetInterfaceForDevice(device);
        Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();
        Ptr<aodv::RoutingProtocol> aodvRouting = routing->GetObject<aodv::RoutingProtocol>();

        if(interfaceStatuses[bestDeviceIndex] == InterfaceStatus::DOWN)
        {
            routing->NotifyInterfaceUp(interface);
            interfaceStatuses[bestDeviceIndex] = InterfaceStatus::UP;
        }
        aodvRouting->PreSendRequest("13.0.0.3", ipv4->GetAddress(interface, 0));
    }
}

double LteRoutingExtension::PredictSignalStrength(uint32_t deviceIndex)
{
    double k = 0.4;
    double g = 0.5;
    double m = 10;

    double weightedAverage = k * devices[deviceIndex]->GetRrc()->GetRsrp() 
                            + (1-k) * (lastWeightedAverage[deviceIndex] + lastTrendValue[deviceIndex]);
    double trendValue = g * (weightedAverage - lastWeightedAverage[deviceIndex]) 
                            + (1-g) * (lastTrendValue[deviceIndex]);
    double prediction = weightedAverage + (m*trendValue);

    lastWeightedAverage[deviceIndex] = weightedAverage;
    lastTrendValue[deviceIndex] = trendValue;

    return prediction;
}

uint32_t LteRoutingExtension::GetBestSignalDeviceIndex()
{
    uint32_t bestIndex = currentlyUsedDeviceIndex;
    double bestRsrq = currentlyUsedDevice->GetRrc()->GetRsrq();

    if(bestRsrq >= 30 && currentlyUsedDevice->GetRrc()->GetRsrp() >= 60)
        return bestIndex;

    for(uint32_t i = 0; i < devices.size(); ++i)
    {   
        double currentRsrq = devices[i]->GetRrc()->GetRsrq();
        
        if(currentRsrq > bestRsrq)
        {
            bestIndex = i;
            bestRsrq = currentRsrq;
        }
    }

    return bestIndex;
}

void LteRoutingExtension::ActivateRoutingOver(uint32_t deviceIndex)
{
    Ptr<LteUeNetDevice> device = devices[deviceIndex];
    uint32_t interface = ipv4->GetInterfaceForDevice(device);
    Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();

    routing->NotifyInterfaceUp(interface);
    interfaceStatuses[deviceIndex] = InterfaceStatus::UP;
}

void LteRoutingExtension::DeactivateRoutingOver(uint32_t deviceIndex)
{
    Ptr<LteUeNetDevice> device = devices[deviceIndex];
    uint32_t interface = ipv4->GetInterfaceForDevice(device);
    Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();

    routing->NotifyInterfaceDown(interface);
    interfaceStatuses[deviceIndex] = InterfaceStatus::DOWN;
}