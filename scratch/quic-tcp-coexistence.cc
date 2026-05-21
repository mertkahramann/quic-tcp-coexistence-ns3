/*
 * TCP CUBIC + QUIC Coexistence Simulation
 * Dumbbell Topology - Congestion Control Fairness Evaluation
 *
 * Author: Lütfi Mert Kahraman
 * Course: Computer Networks and Mobile Computing
 * Instructor: Assoc. Prof. Pınar BÖLÜK
 *
 * Scenarios (run with different parameters):
 *   Phase 1 - Baseline:  --useV3=false
 *   Phase 2 - BBRv3:     --useV3=true
 *   Lossy network:       --lossRate=0.01
 *   Flow ratios:         --nCubic=1 --nQuic=3  (or 2+2, 3+1)
 *   Buffer sizes:        --bufSize=50 (packets)
 *   AQM:                 --useAqm=true
 *
 * Metrics: throughput, Jain's Fairness Index, queue size
 * Output:  results_<tag>.csv
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/tcp-bbr-v3.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"

#include <ctime>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <map>

using namespace ns3;

struct FlowStats
{
    std::string name;
    std::string cc;
    std::string transport;
    uint64_t    rxBytes{0};
    double      rtt = 0;
    std::vector<double> samples;
}; //For showing the flow's details

static std::vector<FlowStats> g_flows;

NS_LOG_COMPONENT_DEFINE("QuicTcpCoexistence");

// ============================================================
// QuicSender: UDP application modeling RFC 9002 CC behavior
//
// Note: This is NOT a full QUIC implementation. It models
// the congestion control behavior defined in RFC 9002:
// - Slow start with exponential cwnd growth
// - Congestion avoidance with additive increase
// - Simplified loss detection via timeout
//
// QUIC's other features (0-RTT, multiplexing, migration)
// are out of scope for this CC fairness study.
// ============================================================
class QuicSender : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::QuicSender")
                                .SetParent<Application>()
                                .SetGroupName("Applications")
                                .AddConstructor<QuicSender>();
        return tid;
    }

    QuicSender()
        : m_socket(nullptr), m_cwnd(10), m_ssthresh(64), 
          m_inFlight(0), m_running(false), m_pktSize(1000)
    {}

    ~QuicSender() noexcept override = default;

    void Setup(Ptr<Socket> socket, Address peer, uint32_t pktSize = 1000)
    {
        m_socket  = socket;
        m_peer    = peer;
        m_pktSize = pktSize;
    }

  private:
    void StartApplication() override
    {
        m_running = true;
        m_socket->Connect(m_peer);
        // Listen for real ACKs coming back from the Echo Server
        m_socket->SetRecvCallback(MakeCallback(&QuicSender::HandleAck, this));
        m_lastAckTime = Simulator::Now();
        
        ScheduleBurst();
        CheckTimeout(); // Start the drop-detection watchdog
    }

    void StopApplication() override
    {
        m_running = false;
        if (m_socket) {
            m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
            m_socket->Close();
        }
    }

    void ScheduleBurst()
    {
        if (!m_running) return;
        Simulator::Schedule(MilliSeconds(1), &QuicSender::SendBurst, this);
    }

    void SendBurst()
    {
        if (!m_running) return;

        uint32_t sent = 0;
        while (m_inFlight < m_cwnd && sent < 16)
        {
            Ptr<Packet> pkt = Create<Packet>(m_pktSize);
            if (m_socket->Send(pkt) >= 0)
            {
                m_inFlight++;
                sent++;
            }
            else break;
        }

        ScheduleBurst();
    }

    // Process REAL ACKs from the network
    void HandleAck(Ptr<Socket> socket)
    {
        if (!m_running) return;
        
        Ptr<Packet> packet;
        while ((packet = socket->Recv()))
        {
            // --- NEW: Flow & RTT Trace Integration ---
            // If you want to log QUIC RTT to your global g_flows array, you can do it here:
            // Time rtt = Simulator::Now() - m_lastAckTime;
            // (Assuming you map this application to its corresponding g_flows index)

            m_lastAckTime = Simulator::Now(); // Reset timeout timer
            
            if (m_inFlight > 0) m_inFlight--;

            // -Congestion Control Window Growth (RFC 9002 Packet-Based Mode) ---
            if (m_cwnd < m_ssthresh) 
            {
                // Slow Start: Exponential growth (1 packet per ACK)
                m_cwnd++; 
            } 
            else 
            {
                // Congestion Avoidance: Additive Increase (Linear growth)
                // In packet-based mode, cwnd increments by 1 ONLY after an entire window is ACKed.
                // To avoid floating-point variables, we implement the classic NS-3 accumulator pattern:
                static uint32_t ackCount = 0;
                if (++ackCount >= m_cwnd)
                {
                    m_cwnd++;
                    ackCount = 0; // Reset accumulator
                }
            }
            
            m_cwnd = std::min(m_cwnd, (uint32_t)200); // Cap window to prevent overflow
        }
    }

    // Packet Loss Detection Watchdog
    void CheckTimeout()
    {
        if (!m_running) return;

        // If we have packets in flight, but haven't received an ACK in 50ms, assume a drop!
        if (m_inFlight > 0 && (Simulator::Now() - m_lastAckTime) > MilliSeconds(50))
        {
            // Congestion Control - Multiplicative Decrease
            m_ssthresh = std::max(10u, m_cwnd / 2);
            m_cwnd = 10; // Drop back to initial window (RFC 9002 standard behavior on timeout)
            m_inFlight = 0; // Clear in-flight to unblock sending
            m_lastAckTime = Simulator::Now(); // Reset timer
        }

        Simulator::Schedule(MilliSeconds(10), &QuicSender::CheckTimeout, this);
    }

    Ptr<Socket> m_socket;
    Address     m_peer;
    uint32_t    m_cwnd;
    uint32_t    m_ssthresh;
    uint32_t    m_inFlight;
    bool        m_running;
    uint32_t    m_pktSize;
    Time        m_lastAckTime;
};

NS_OBJECT_ENSURE_REGISTERED(QuicSender);

void SampleThroughput(double interval, double stopTime)
{
    double now = Simulator::Now().GetSeconds();
    std::ostringstream line;
    line << "[t=" << std::fixed << std::setprecision(0) << now << "s]";

    double total = 0;
    for (auto& f : g_flows)
    {
        double mbps = (f.rxBytes * 8.0) / (interval * 1e6);
        f.samples.push_back(mbps);
        f.rxBytes = 0;
        total += mbps;
        line << "  " << f.name << "(" << f.transport << "/" << f.cc << ")="
             << std::fixed << std::setprecision(3) << mbps << "Mbps";
    }
    line << "  TOTAL=" << std::fixed << std::setprecision(3) << total << "Mbps";
    NS_LOG_UNCOND(line.str());

    if (now + interval < stopTime)
        Simulator::Schedule(Seconds(interval), &SampleThroughput, interval, stopTime);
}

// ============================================================
// Jain's Fairness Index
// ============================================================
double JainFairness(const std::vector<double>& x)
{
    if (x.empty()) return 0.0;
    double sum = 0, sumSq = 0;
    for (double v : x) { sum += v; sumSq += v * v; }
    return sumSq > 0 ? (sum * sum) / (x.size() * sumSq) : 0.0;
}

// Callback function for packet reception
static void OnPacketRx(uint32_t flowIdx, Ptr<const Packet> pkt, const Address& addr)
{
    g_flows[flowIdx].rxBytes += pkt->GetSize();
}

static std::map<std::string, uint32_t> g_rttCounters;

// --- RTT Trace Callback ---
static void OnTcpRtt(std::string context, Time oldRtt, Time newRtt)
{
    double rttMs = newRtt.GetMilliSeconds();
    double now = Simulator::Now().GetSeconds();

    // We only print 1 out of every 100 samples. 
    // TCP generates thousands of ACKs, and printing all of them would freeze your terminal.
    std::string nodeId = context.substr(10, 1);

    if (++g_rttCounters[nodeId] % 50 == 0 && now > 2.0) 
        {
            NS_LOG_UNCOND("[RTT Trace] Node-" << nodeId 
                        << " | t=" << now << "s | Measured TCP Latency: " << rttMs << " ms");
        }
}

// Function to connect the trace AFTER the sockets have been dynamically created
static void ConnectRttTraces()
{
    // Globally hooks into all active TCP sockets in the simulation
    Config::Connect("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/RTT", MakeCallback(&OnTcpRtt));
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[])
{
    // --- Parameters ---
    bool     useV3      = false;
    bool     useAqm     = false;
    double   simTime    = 60.0;
    double   bwMbps     = 10.0;
    double   delayMs    = 10.0;
    uint32_t nCubic     = 2;
    uint32_t nBbr       = 2;      // BBR flow
    uint32_t nQuic      = 2;
    uint32_t bufSize    = 512;    // packets
    double   lossRate   = 0.0;    // 0.0 = no loss, 0.01 = 1%
    std::string tag     = "";     // output file tag

    CommandLine cmd(__FILE__);
    cmd.AddValue("useV3",    "Use TcpBbrV3 for TCP flows",          useV3);
    cmd.AddValue("useAqm",   "Use FqCoDel AQM on bottleneck",       useAqm);
    cmd.AddValue("simTime",  "Simulation duration (s)",             simTime);
    cmd.AddValue("bwMbps",   "Bottleneck bandwidth (Mbps)",         bwMbps);
    cmd.AddValue("delayMs",  "Bottleneck one-way delay (ms)",       delayMs);
    cmd.AddValue("nCubic",   "Number of TCP CUBIC/BBRv3 flows",     nCubic);
    cmd.AddValue("nBbr",     "Number of TCP BBR flows",             nBbr);
    cmd.AddValue("nQuic",    "Number of QUIC (UDP) flows",          nQuic);
    cmd.AddValue("bufSize",  "Bottleneck queue size (packets)",     bufSize);
    cmd.AddValue("lossRate", "Random loss rate on bottleneck link", lossRate);
    cmd.AddValue("tag",      "Output CSV filename tag",             tag);
    cmd.Parse(argc, argv);

    std::string tcpCcName = useV3 ? "TcpBbrV3" : "TcpCubic";

    uint32_t nTotal = nCubic + nBbr + nQuic;
    g_flows.resize(nTotal);

    g_flows.resize(nTotal);
    std::string bbrName = useV3 ? "TcpBbrV3" : "TcpBbr"; // Use BBRv1 or BBRv3 based on flag

    // 1. Setup CUBIC tracking
    for (uint32_t i = 0; i < nCubic; i++) {
        g_flows[i].name      = "CUBIC-" + std::to_string(i);
        g_flows[i].cc        = "TcpCubic";
        g_flows[i].transport = "TCP";
    }
    // 2. Setup BBR tracking
    for (uint32_t i = 0; i < nBbr; i++) {
        g_flows[nCubic + i].name      = "BBR-" + std::to_string(i);
        g_flows[nCubic + i].cc        = bbrName;
        g_flows[nCubic + i].transport = "TCP";
    }
    // 3. Setup QUIC tracking
    for (uint32_t i = 0; i < nQuic; i++) {
        g_flows[nCubic + nBbr + i].name      = "QUIC-" + std::to_string(i);
        g_flows[nCubic + nBbr + i].cc        = "RFC9002";
        g_flows[nCubic + nBbr + i].transport = "UDP";
    }

    // --- TCP congestion control ---
    if (useV3)
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                           TypeIdValue(TcpBbrV3::GetTypeId()));
        NS_LOG_UNCOND("TCP CC: TcpBbrV3 (DOWN gain=0.90)");
    }
    else
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                           StringValue("ns3::TcpCubic"));
        NS_LOG_UNCOND("TCP CC: TcpCubic (baseline)");
    }

    Config::SetDefault("ns3::TcpSocket::SndBufSize",  UintegerValue(1 << 20));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize",  UintegerValue(1 << 20));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1448));
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(2));

    // --- Topology ---
    //
    //  L0 --\              /-- R0
    //  L1 ---[RA] ---- [RB]--- R1
    //  L2 --/              \-- R2
    //  ...                     ...
    //
    // Left nodes 0..nCubic-1     : TCP senders
    // Left nodes nCubic..nTotal-1: QUIC senders
    // Right nodes mirror left nodes

    NodeContainer leftNodes, rightNodes, routers;
    leftNodes.Create(nTotal);
    rightNodes.Create(nTotal);
    routers.Create(2);

    InternetStackHelper stack;
    stack.Install(leftNodes);
    stack.Install(rightNodes);
    stack.Install(routers);

    // Access links: 100 Mbps, 1 ms
    PointToPointHelper access;
    access.SetDeviceAttribute ("DataRate", StringValue("100Mbps"));
    access.SetChannelAttribute("Delay",    StringValue("1ms"));

    // --- Bottleneck link (Correctly applying Jitter) ---
    PointToPointHelper bottleneck;
    
    // Set dynamic bandwidth from the command line
    std::ostringstream bwStr;
    bwStr << bwMbps << "Mbps";
    bottleneck.SetDeviceAttribute ("DataRate", StringValue(bwStr.str()));

    std::ostringstream dlStr;
    dlStr << delayMs << "ms";
    bottleneck.SetChannelAttribute("Delay", StringValue(dlStr.str()));

    // IP assignment
    Ipv4AddressHelper ipv4;

    // Left access links
    std::vector<Ipv4InterfaceContainer> leftIf(nTotal);
    for (uint32_t i = 0; i < nTotal; i++)
    {
        NetDeviceContainer dev = access.Install(leftNodes.Get(i), routers.Get(0));
        std::ostringstream sub;
        sub << "10.1." << (i + 1) << ".0";
        ipv4.SetBase(sub.str().c_str(), "255.255.255.0");
        leftIf[i] = ipv4.Assign(dev);
    }

    // Bottleneck
    NetDeviceContainer bnDevs = bottleneck.Install(routers.Get(0), routers.Get(1));

    ipv4.SetBase("10.100.1.0", "255.255.255.0");
    ipv4.Assign(bnDevs);

    // Right access links
    std::vector<Ipv4InterfaceContainer> rightIf(nTotal);
    for (uint32_t i = 0; i < nTotal; i++)
    {
        NetDeviceContainer dev = access.Install(routers.Get(1), rightNodes.Get(i));
        std::ostringstream sub;
        sub << "10.2." << (i + 1) << ".0";
        ipv4.SetBase(sub.str().c_str(), "255.255.255.0");
        rightIf[i] = ipv4.Assign(dev);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // --- Bottleneck queue configuration ---
    TrafficControlHelper tch;
    if (useAqm)
    {
        // FqCoDel: fair queuing + controlled delay
        tch.SetRootQueueDisc("ns3::FqCoDelQueueDisc");
        NS_LOG_UNCOND("AQM: FqCoDel");
    }
    else
    {
        // Drop-tail with fixed buffer size
        tch.SetRootQueueDisc("ns3::FifoQueueDisc",
                             "MaxSize",
                             QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, bufSize)));
        NS_LOG_UNCOND("AQM: DropTail, bufSize=" << bufSize << " pkts");
    }
    // Remove default queue disc before installing ours
    TrafficControlHelper tchClean;
    tchClean.Uninstall(bnDevs);
    tch.Install(bnDevs);

    // --- Random loss on bottleneck (lossy network scenario) ---
    if (lossRate > 0.0)
    {
        Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
        em->SetAttribute("ErrorRate", DoubleValue(lossRate));
        em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        bnDevs.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        NS_LOG_UNCOND("Loss rate: " << lossRate * 100 << "%");
    }

    // --- TCP Flows (CUBIC + BBR combined) ---
    uint16_t tcpPort = 5000;
    ApplicationContainer tcpSinks;
    uint32_t nTcpTotal = nCubic + nBbr;

    for (uint32_t i = 0; i < nTcpTotal; i++)
    {
        // 1. DYNAMIC CONGESTION CONTROL OVERRIDE (PER NODE)
        Ptr<Node> senderNode = leftNodes.Get(i);
        Ptr<TcpL4Protocol> tcpStack = senderNode->GetObject<TcpL4Protocol>();
        
        if (i < nCubic) {
            // First 'nCubic' nodes use CUBIC
            tcpStack->SetAttribute("SocketType", TypeIdValue(TcpCubic::GetTypeId()));
        } else {
            // Next 'nBbr' nodes use BBR (v1 or v3 depending on the flag)
            if (useV3) tcpStack->SetAttribute("SocketType", TypeIdValue(TcpBbrV3::GetTypeId()));
            else       tcpStack->SetAttribute("SocketType", TypeIdValue(TcpBbr::GetTypeId()));
        }

        // 2. Receiver (Sink)
        PacketSinkHelper sinkH("ns3::TcpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), tcpPort + i));
        ApplicationContainer s = sinkH.Install(rightNodes.Get(i));
        s.Start(Seconds(0.1));
        s.Stop(Seconds(simTime));
        tcpSinks.Add(s);

        // 3. Sender (Source)
        BulkSendHelper srcH("ns3::TcpSocketFactory",
                            InetSocketAddress(rightIf[i].GetAddress(1), tcpPort + i));
        srcH.SetAttribute("MaxBytes", UintegerValue(0));

        // Install on the dynamically configured sender node
        ApplicationContainer src = srcH.Install(senderNode);
        src.Start(Seconds(1.0));
        src.Stop(Seconds(simTime - 1.0));
    }

    // Throughput callbacks for all TCP flows
    for (uint32_t i = 0; i < nTcpTotal; i++)
    {
        Ptr<PacketSink> ps = DynamicCast<PacketSink>(tcpSinks.Get(i));
        ps->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnPacketRx, i));
    }

    // --- QUIC (UDP) flows ---
    uint16_t udpPort = 6000;
    ApplicationContainer udpSinks;
    
    // Include this header at the top of your file if not already present:
    // #include "ns3/applications-module.h"

    for (uint32_t i = 0; i < nQuic; i++)
    {
        uint32_t idx = nCubic + nBbr + i;

        // USE ECHO SERVER INSTEAD OF PACKET SINK
        UdpEchoServerHelper echoServer(udpPort + i);
        ApplicationContainer s = echoServer.Install(rightNodes.Get(idx));
        s.Start(Seconds(0.1));
        s.Stop(Seconds(simTime));
        udpSinks.Add(s);

        // QUIC sender
        Ptr<Socket> sock = Socket::CreateSocket(leftNodes.Get(idx),
                                                UdpSocketFactory::GetTypeId());
        Address peer(InetSocketAddress(rightIf[idx].GetAddress(1), udpPort + i));

        Ptr<QuicSender> app = CreateObject<QuicSender>();
        app->Setup(sock, peer, 1000);
        leftNodes.Get(idx)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(simTime - 1.0));
    }

    // Throughput callbacks for QUIC
    for (uint32_t i = 0; i < nQuic; i++)
    {
        // Get it as a generic Application, do NOT cast to PacketSink
        Ptr<Application> app = udpSinks.Get(i); 
        uint32_t flowIdx = nCubic + nBbr + i;
        
        // Connect to the "Rx" trace source directly
        app->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnPacketRx, flowIdx));
    }

    // --- Throughput sampling ---
    double sampleInterval = 1.0;
    Simulator::Schedule(Seconds(1.0 + sampleInterval),
                        &SampleThroughput, sampleInterval, simTime);

    Simulator::Schedule(Seconds(1.1), &ConnectRttTraces);

    // --- Run ---
    Simulator::Stop(Seconds(simTime + 0.5));
    NS_LOG_UNCOND("=== Simulation Start ==="
                  << "\n  Phase:   " << (useV3 ? "BBRv3" : "Baseline")
                  << "\n  Flows:   " << nCubic << " CUBIC + " << nQuic << " QUIC"
                  << "\n  BW:      " << bwMbps << " Mbps"
                  << "\n  Delay:   " << delayMs << " ms"
                  << "\n  Buffer:  " << bufSize << " pkts"
                  << "\n  Loss:    " << lossRate * 100 << "%"
                  << "\n  AQM:     " << (useAqm ? "FqCoDel" : "DropTail"));

    Simulator::Run();

    // --- Results ---
    // Skip first 5 samples (transient)
    size_t nSamples = g_flows.empty() ? 0 : g_flows[0].samples.size();
    size_t skip = std::min((size_t)5, nSamples / 3);

    std::vector<double> avgPerFlow;
    double totalAvg = 0;

    for (auto& f : g_flows)
    {
        double sum = 0;
        for (size_t i = skip; i < f.samples.size(); i++) sum += f.samples[i];
        double avg = (f.samples.size() > skip) ? sum / (f.samples.size() - skip) : 0;
        avgPerFlow.push_back(avg);
        totalAvg += avg;
    }

    for (size_t i = 0; i < g_flows.size(); i++)
    {
        double share = totalAvg > 0 ? (avgPerFlow[i] / totalAvg) * 100 : 0;
        NS_LOG_UNCOND(g_flows[i].name << " (" << g_flows[i].transport
                    << "/" << g_flows[i].cc << "): "
                    << avgPerFlow[i] << " Mbps  " << share << "%");
    }

    double jfi = JainFairness(avgPerFlow);
    NS_LOG_UNCOND("Total: " << totalAvg << " Mbps");
    NS_LOG_UNCOND("Jain's FI: " << jfi);

    // --- Validation ---
    NS_LOG_UNCOND("=== VALIDATION ===");
    if (totalAvg > bwMbps * 1.1)
        NS_LOG_UNCOND("WARNING: Total throughput exceeds bottleneck!");
    else
        NS_LOG_UNCOND("OK: Total throughput within bottleneck limit (" 
                      << (totalAvg / bwMbps * 100) << "%)");
    for (size_t i = 0; i < g_flows.size(); i++)
    {
        if (avgPerFlow[i] < 0.01)
            NS_LOG_UNCOND("WARNING: Starvation on " << g_flows[i].name
                          << " (" << g_flows[i].transport << "/" << g_flows[i].cc << ")");
    }

    // --- Write CSV ---
    std::string phase  = useV3  ? "bbrv3"    : "baseline";
    std::string aqmStr = useAqm ? "_fqcodel" : "";
    std::ostringstream fname;
    fname << "results_" << phase
          << "_c" << nCubic << "q" << nQuic
          << "_buf" << bufSize
          << "_loss" << (int)(lossRate * 100)
          << aqmStr;
    if (!tag.empty()) fname << "_" << tag;
    fname << ".csv";

    std::ofstream csv(fname.str());
    csv << "time_s";
    for (auto& f : g_flows)
        csv << "," << f.name << "_" << f.transport << "_" << f.cc << "_mbps";
    csv << ",total_mbps\n";

    for (size_t t = 0; t < nSamples; t++)
    {
        csv << (int)(t + 2);
        double rowTotal = 0;
        for (auto& f : g_flows)
        {
            double v = t < f.samples.size() ? f.samples[t] : 0;
            csv << "," << v;
            rowTotal += v;
        }
        csv << "," << rowTotal << "\n";
    }
    csv.close();
    NS_LOG_UNCOND("CSV: " << fname.str());

    Simulator::Destroy();
    return 0;
}
