/*
 * TCP BBRv3 - Modified pacing gain for improved fairness
 * Subclass of TcpBbr with DOWN phase gain: 0.75 -> 0.90
 *
 * Author: Lütfi Mert Kahraman
 */

#ifndef TCPBBRV3_H
#define TCPBBRV3_H

#include "tcp-bbr.h"

namespace ns3
{

class TcpBbrV3 : public TcpBbr
{
  public:
    static const double PACING_GAIN_CYCLE_V3[];

    static TypeId GetTypeId();
    TcpBbrV3();
    TcpBbrV3(const TcpBbrV3& sock);

    std::string GetName() const override;
    Ptr<TcpCongestionOps> Fork() override;
    void AdvanceCyclePhase() override;
};

} // namespace ns3
#endif // TCPBBRV3_H