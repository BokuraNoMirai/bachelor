#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"

using namespace ns3;

enum class InterfaceStatus
{
    DOWN,
    UP
};

class LteRoutingExtension
{
    public:

    LteRoutingExtension();
    ~LteRoutingExtension();

    void Install(Ptr<Node> pNode);
    void Start();
    void AddDevice(Ptr<NetDevice> netDevice);
    void SetInterval(double pInterval);
    void CheckSignalStrength();

    private:

    uint32_t GetBestSignalDeviceIndex();
    void ActivateRoutingOver(uint32_t deviceIndex);
    void DeactivateRoutingOver(uint32_t deviceIndex);
    void PreSendRouteRequest();
    double PredictSignalStrength(uint32_t deviceIndex);

    Ptr<Node> node;
    Ptr<Ipv4> ipv4;
    std::vector<Ptr<LteUeNetDevice>> devices;
    std::vector<InterfaceStatus> interfaceStatuses;
    std::vector<double> lastWeightedAverage;
    std::vector<double> lastTrendValue;
    Ptr<LteUeNetDevice> currentlyUsedDevice;
    uint32_t currentlyUsedDeviceIndex;

    double interval;
};