/*
 * TCP BBRv3 - Modified pacing gain for improved fairness
 * DOWN phase pacing gain: 0.75 -> 0.90
 * Reference: Piotrowska (2024), Section 3.3
 *
 * Author: Lütfi Mert Kahraman
 */

#include "tcp-bbr-v3.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpBbrV3");
NS_OBJECT_ENSURE_REGISTERED(TcpBbrV3);

// BBRv3 pacing gain cycle
// UP=1.25 (same), DOWN=0.90 (was 0.75), CRUISE=1.0 x6
const double TcpBbrV3::PACING_GAIN_CYCLE_V3[] = {5.0 / 4, 0.9, 1, 1, 1, 1, 1, 1};

TypeId
TcpBbrV3::GetTypeId()
{
    static TypeId tid = TypeId("ns3::TcpBbrV3")
                            .SetParent<TcpBbr>()
                            .SetGroupName("Internet")
                            .AddConstructor<TcpBbrV3>();
    return tid;
}

TcpBbrV3::TcpBbrV3()
    : TcpBbr()
{
}

TcpBbrV3::TcpBbrV3(const TcpBbrV3& sock)
    : TcpBbr(sock)
{
}

std::string
TcpBbrV3::GetName() const
{
    return "TcpBbrV3";
}

Ptr<TcpCongestionOps>
TcpBbrV3::Fork()
{
    return CopyObject<TcpBbrV3>(this);
}

void
TcpBbrV3::AdvanceCyclePhase()
{
    NS_LOG_FUNCTION(this);
    m_cycleStamp = Simulator::Now();
    m_cycleIndex = (m_cycleIndex + 1) % GAIN_CYCLE_LENGTH;
    m_pacingGain = PACING_GAIN_CYCLE_V3[m_cycleIndex];
}

} // namespace ns3