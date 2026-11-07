//
// G1_VirtualPhone — scripted BLE central for the emulated G1 arm.
//
// Talks to the arm only through the NRF5340_RADIO model (same hooks as
// g1-fw's BLE_VirtualCentral): connects to the advertiser, then runs a small
// link layer (SN/NESN, MD, control PDUs, L2CAP fragmentation/reassembly) and
// ATT client/server so that full-size NUS and SMP writes reach the firmware.
//
// No pairing or encryption is attempted: SMP requests from the arm are ignored.
//
// Monitor API:
//   vphone Enabled true
//   vphone LoadScript @/path/script.txt
//   vphone Status / vphone Finished / vphone NotificationCount / vphone LastNotification
//
// Script lines (handles may be NUS_RX, NUS_TX, SMP or a number):
//   mtu <n>                 ATT Exchange MTU
//   discover                read all characteristic declarations
//   subscribe <h>           write 01 00 to the CCC after characteristic <h>
//   w <h> <hex>             write command; continue when all fragments are acked
//   wn <h> <hex> [ms]       write command, then wait for one notification (or ms)
//   wr <h> <hex>            write request (logs the ATT response or error)
//   wait <ms>               wait virtual time
//   reconnect [ms]          wait until the arm has reset/advertised again and we reconnected
//   note <text>             log a marker
//
// Bridge mode (milestone 2): `vphone BridgePath @/path/sock` makes the phone a lock-step proxy for an
// external host program (tools/g1host.py). At every connection event (and every 10 ms of virtual
// time while disconnected) the phone sends the host one batch and blocks until the host answers:
//   phone -> host:  T <virtual us> <connected 0|1> <mtu> <step index> <step count>
//                   H <name> <handle hex>        (once per discovered characteristic)
//                   N <handle hex> <hex>         (notifications since the last batch)
//                   L <text>                     (link events)
//                   E                            (end of batch)
//   host -> phone:  W <handle|name> <hex>        (ATT write command, queued at once)
//                   X <script line>              (append a script step: mtu, discover, subscribe, wr, wait, ...)
//                   D <ms>                       (stay silent for ms: the arm sees a lost link)
//                   F <peripheral> <property> <value>   (set a model property, fault injection)
//                   C <peripheral> <method> [arg]       (call a model method, e.g. DumpFrame)
//                   M <hex address> <hex length> <path>  (save a bus memory range to a file)
//                   Z                            (machine reset)
//                   A on|off | error <hex> | ns <hex> | ds <hex>   (the phone's ANCS, below)
//                   G                            (go: continue the emulation)
//
// ANCS (VM.md §6.12): the GATT server of an iPhone that shares its notifications, for the relay's GATT client. Off, it is
// a phone without ANCS. `error` makes writes to the Control Point fail with that ATT error (0: they succeed); `ns` and `ds`
// send a notification whether subscribed or not, so that a test sees what the arm does with one it didn't ask for. The
// host hears what the arm does as link events: `ancs request <op>`, `ancs ccc ns|ds <value>`, `ancs cp <hex>`.
//
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Wireless
{
    public class G1_VirtualPhone : BasicDoubleWordPeripheral, IKnownSize
    {
        public G1_VirtualPhone(IMachine machine, NRF5340_RADIO radio) : base(machine)
        {
            this.radio = radio;
            radio.FrameSent += OnFrameSent;
            radio.RxWindowOpened += OnRxWindowOpened;
            tickTimer = new LimitTimer(machine.ClockSource, 1000, this, "vphoneTick", 10,
                Antmicro.Renode.Time.Direction.Ascending, enabled: false, workMode: WorkMode.Periodic, eventEnabled: true);
            tickTimer.LimitReached += OnTick;
        }

        public long Size => 0x1000;

        // Test aid (v0_tests pmic_install): halt the app core for this many virtual seconds right after
        // the next machine reset, i.e. at MCUboot's start, as a long MCUboot install does on silicon
        // (flash erases are instant here). One-shot.
        public double StallAppCoreAfterResetS { get; set; }

        public override void Reset()
        {
            base.Reset();
            var stall = StallAppCoreAfterResetS;
            if(stall <= 0)
            {
                return;
            }
            StallAppCoreAfterResetS = 0;
            machine.ScheduleAction(TimeInterval.FromMicroseconds(1), _ =>
            {
                if(!machine.TryGetByName("sysbus.cpuapp", out IPeripheral p) || !(p is ICPU cpu))
                {
                    this.Log(LogLevel.Warning, "vphone: no app core to stall");
                    return;
                }
                cpu.IsHalted = true;
                this.Log(LogLevel.Info, "vphone: app core halted for {0} s after the reset (a long MCUboot install)", stall);
                machine.ScheduleAction(TimeInterval.FromMicroseconds((ulong)(stall * 1e6)), __ =>
                {
                    cpu.IsHalted = false;
                    this.Log(LogLevel.Info, "vphone: app core released");
                });
            });
        }

        public bool Enabled { get; set; }
        public int IntervalUnits { get; set; } = 6;
        public int MaxPdusPerEvent { get; set; } = 6;
        public bool UseDle { get; set; } = true;
        // Send one SCAN_REQ before each connection; ADV_IND data and the SCAN_RSP go to the bridge.
        public bool ScanFirst { get; set; }
        public int HeartbeatMs { get; set; }
        public bool Connected => connected;
        public bool Finished => steps.Count > 0 && stepIndex >= steps.Count;
        public int NotificationCount => notifications.Count;
        public string LastNotification => notifications.Count > 0 ? notifications[notifications.Count - 1] : "";
        public int AttMtu => attMtu;

        public string Status =>
            $"connected={connected} step={stepIndex}/{steps.Count} mtu={attMtu} nus_rx=0x{Handle("NUS_RX"):X} nus_tx=0x{Handle("NUS_TX"):X} smp=0x{Handle("SMP"):X} " +
            $"tx_pdus={txPdus} rx_pdus={rxPdus} notifications={notifications.Count} events={events} link_losses={linkLosses} injected={injected} peer_responses={peerResponses} naks={naks}";

        public string Notifications(int last)
        {
            return string.Join("\n", notifications.Skip(Math.Max(0, notifications.Count - last)));
        }

        public void LoadScript(string path)
        {
            foreach(var raw in File.ReadAllLines(path))
            {
                AddStep(raw);
            }
            this.Log(LogLevel.Info, "vphone: {0} script steps loaded from {1}", steps.Count, path);
        }

        public void AddStep(string line)
        {
            line = line.Trim();
            if(line.Length == 0 || line.StartsWith("#"))
            {
                return;
            }
            steps.Add(line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries));
        }

        public string BridgePath
        {
            get => bridgePath;
            set
            {
                bridgePath = value;
                if(System.IO.File.Exists(value))
                {
                    System.IO.File.Delete(value);
                }
                listener = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
                listener.Bind(new UnixDomainSocketEndPoint(value));
                listener.Listen(1);
                tickTimer.Enabled = true;
                this.Log(LogLevel.Info, "vphone: bridge listening on {0}", value);
            }
        }

        public int BridgeTimeoutSeconds { get; set; } = 120;

        private void OnTick()
        {
            if(Enabled && (!connected || Now() < silentUntil))
            {
                BridgeSync();
            }
        }

        // One lock-step exchange with the host program; returns when the host says G.
        private void BridgeSync()
        {
            if(listener == null)
            {
                return;
            }
            if(bridge == null)
            {
                if(!listener.Poll(0, SelectMode.SelectRead))
                {
                    return;
                }
                bridge = listener.Accept();
                bridgeStream = new NetworkStream(bridge);
                bridgeReader = new System.IO.StreamReader(bridgeStream, Encoding.ASCII);
                bridgeWriter = new System.IO.StreamWriter(bridgeStream, Encoding.ASCII) { AutoFlush = false, NewLine = "\n" };
                handlesSent.Clear();
                this.Log(LogLevel.Info, "vphone: bridge host attached");
            }
            try
            {
                var sb = new StringBuilder();
                sb.Append($"T {(long)(Now() * 1000)} {(connected ? 1 : 0)} {attMtu} {stepIndex} {steps.Count}\n");
                foreach(var kv in handles)
                {
                    if(!handlesSent.Contains(kv.Key))
                    {
                        handlesSent.Add(kv.Key);
                        sb.Append($"H {kv.Key} {kv.Value:X}\n");
                    }
                }
                foreach(var n in bridgeNotifications)
                {
                    sb.Append(n).Append('\n');
                }
                bridgeNotifications.Clear();
                foreach(var l in bridgeLinkEvents)
                {
                    sb.Append("L ").Append(l).Append('\n');
                }
                bridgeLinkEvents.Clear();
                sb.Append("E\n");
                bridgeWriter.Write(sb.ToString());
                bridgeWriter.Flush();
                bridge.ReceiveTimeout = BridgeTimeoutSeconds * 1000;
                while(true)
                {
                    var line = bridgeReader.ReadLine();
                    if(line == null)
                    {
                        throw new System.IO.IOException("host closed the bridge");
                    }
                    if(line == "G")
                    {
                        break;
                    }
                    BridgeCommand(line);
                }
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning, "vphone: bridge detached ({0})", e.Message);
                bridge.Dispose();
                bridge = null;
            }
        }

        private void BridgeCommand(string line)
        {
            var parts = line.Split(new[] { ' ' }, 4, StringSplitOptions.RemoveEmptyEntries);
            if(parts.Length == 0)
            {
                return;
            }
            switch(parts[0])
            {
            case "W":
                {
                    var h = Resolve(parts[1]);
                    QueueL2cap(4, new byte[] { 0x52, (byte)h, (byte)(h >> 8) }.Concat(ParseHex(parts[2])).ToArray());
                    break;
                }
            case "X":
                AddStep(line.Substring(2));
                break;
            case "D":
                silentUntil = Now() + double.Parse(parts[1]);
                this.Log(LogLevel.Info, "vphone: silent for {0} ms (link drop test)", parts[1]);
                break;
            case "F":
                SetModelProperty(parts[1], parts[2], parts.Length > 3 ? parts[3] : "");
                break;
            case "C":
                CallModelMethod(parts[1], parts[2], parts.Length > 3 ? parts[3] : null);
                break;
            case "M":
                try
                {
                    var bytes = machine.SystemBus.ReadBytes(Convert.ToUInt64(parts[1], 16), Convert.ToInt32(parts[2], 16));
                    System.IO.File.WriteAllBytes(parts[3], bytes);
                    bridgeLinkEvents.Add($"result M {bytes.Length} bytes to {parts[3]}");
                }
                catch(Exception e)
                {
                    bridgeLinkEvents.Add($"result M FAILED {e.Message}");
                }
                break;
            case "Z":
                this.Log(LogLevel.Info, "vphone: machine reset requested by the bridge host");
                machine.RequestReset();
                break;
            case "A":
                AncsCommand(parts);
                break;
            default:
                this.Log(LogLevel.Warning, "vphone: unknown bridge command {0}", line);
                break;
            }
        }

        private object FindModel(string path)
        {
            if(machine.TryGetByName(path, out IPeripheral p))
            {
                return p;
            }
            this.Log(LogLevel.Warning, "vphone: no peripheral {0}", path);
            return null;
        }

        private void SetModelProperty(string path, string prop, string value)
        {
            var p = FindModel(path);
            var pi = p?.GetType().GetProperty(prop);
            if(pi == null)
            {
                this.Log(LogLevel.Warning, "vphone: no property {0}.{1}", path, prop);
                return;
            }
            try
            {
                pi.SetValue(p, Convert.ChangeType(value, pi.PropertyType, System.Globalization.CultureInfo.InvariantCulture));
                this.Log(LogLevel.Info, "vphone: set {0}.{1} = {2}", path, prop, value);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning, "vphone: setting {0}.{1} failed: {2}", path, prop, (e.InnerException ?? e).Message);
            }
        }

        private void CallModelMethod(string path, string method, string arg)
        {
            var p = FindModel(path);
            var mi = p?.GetType().GetMethods().FirstOrDefault(m => m.Name == method && m.GetParameters().Length == (arg == null ? 0 : 1));
            if(mi == null)
            {
                this.Log(LogLevel.Warning, "vphone: no method {0}.{1}", path, method);
                return;
            }
            var args = arg == null ? new object[0] : new object[] { Convert.ChangeType(arg, mi.GetParameters()[0].ParameterType, System.Globalization.CultureInfo.InvariantCulture) };
            try
            {
                var result = mi.Invoke(p, args);
                bridgeLinkEvents.Add($"result {method} {result}");
            }
            catch(Exception e)
            {
                bridgeLinkEvents.Add($"result {method} FAILED {(e.InnerException ?? e).Message}");
            }
        }

        // ---------------- radio callbacks ----------------

        private void OnFrameSent(IRadio sender, byte[] frame)
        {
            if(!Enabled || frame == null || frame.Length < 6)
            {
                return;
            }
            if(Same(frame, AdvAccessAddress) && (frame[4] & 0x0F) == 0x0 && frame.Length >= 4 + 2 + 6)
            {
                if(connected)
                {
                    LinkLost();
                }
                advA = new byte[6];
                Array.Copy(frame, 6, advA, 0, 6);
                var adv = AdvPayload(frame);
                if(ScanFirst && adv != lastAdv)
                {
                    lastAdv = adv;
                    bridgeLinkEvents.Add($"adv {adv}");
                }
                return;
            }
            if(Same(frame, AdvAccessAddress) && (frame[4] & 0x0F) == 0x4 && frame.Length >= 4 + 2 + 6)
            {
                bridgeLinkEvents.Add($"scanrsp {AdvPayload(frame)}");
                return;
            }
            if(!connected)
            {
                return;
            }
            if(!Same(frame, DataAccessAddress))
            {
                return;
            }
            var hdr = frame[4];
            var len = frame[5];
            if(frame.Length < 6 + len)
            {
                return;
            }
            var llid = hdr & 0x3;
            var pNesn = (hdr >> 2) & 1;
            var pSn = (hdr >> 3) & 1;
            peerMd = ((hdr >> 4) & 1) != 0;
            peerResponses++;
            if(inflight != null && inflight.Length > 0 && pNesn == txSn)
            {
                naks++;
            }
            if(pNesn != txSn)
            {
                txSn ^= 1;
                if(inflight != null && inflight.Length > 0)
                {
                    txPdus++;
                }
                inflight = null;
                OnPduAcked();
            }
            if(pSn == nesn)
            {
                nesn ^= 1;
                if(len > 0)
                {
                    rxPdus++;
                    var payload = new byte[len];
                    Array.Copy(frame, 6, payload, 0, len);
                    HandlePeerPdu(llid, payload);
                }
            }
        }

        private void OnRxWindowOpened()
        {
            if(!Enabled)
            {
                return;
            }
            var aa = radio.OnAirAddressPublic(0);
            if(aa == null || aa.Length < 4)
            {
                return;
            }
            if(Now() < silentUntil)
            {
                return;
            }
            if(!connected && advA != null && Same(aa, AdvAccessAddress))
            {
                if(ScanFirst && !scanned)
                {
                    scanned = true;
                    SendScanReq();
                    return;
                }
                SendConnectInd();
                return;
            }
            if(!Same(aa, DataAccessAddress))
            {
                return;
            }
            if(!connected)
            {
                connected = true;
                maxTx = MaxLlPayload;
                if(UseDle)
                {
                    QueueControl(0x14, 251, 0, 0x48, 0x08, 251, 0, 0x48, 0x08);
                }
                this.Log(LogLevel.Info, "vphone: connected");
                bridgeLinkEvents.Add($"connected {Now():F1}");
            }
            var now = Now();
            if(now - lastEventStart > IntervalUnits * 1.25 * 0.5)
            {
                lastEventStart = now;
                pdusThisEvent = 0;
                events++;
                Advance();
            }
            pdusThisEvent++;
            injected++;
            SendDataPdu();
        }

        // ---------------- link layer ----------------

        // The arm is advertising again while we think we are connected: it reset or dropped the link.
        private void LinkLost()
        {
            connected = false;
            scanned = false;
            lastAdv = null;
            inflight = null;
            controlQueue.Clear();
            dataQueue.Clear();
            rxL2cap.Clear();
            attPending = false;
            attMtu = 23;
            handles.Clear();
            characteristics.Clear();
            handlesSent.Clear();
            linkLosses++;
            bridgeLinkEvents.Add($"lost {Now():F1}");
            this.Log(LogLevel.Info, "vphone: link lost at t={0:F1}ms (arm advertising again), reconnecting", Now());
        }

        private static string AdvPayload(byte[] frame)
        {
            var len = Math.Min((int)frame[5], frame.Length - 6);
            return len > 6 ? Hex(frame.Skip(12).Take(len - 6).ToArray()) : "";
        }

        private void SendScanReq()
        {
            var pdu = new List<byte>();
            pdu.AddRange(AdvAccessAddress);
            pdu.Add(0x83);
            pdu.Add(12);
            pdu.AddRange(InitA);
            pdu.AddRange(advA);
            Inject(pdu);
            this.Log(LogLevel.Info, "vphone: SCAN_REQ sent");
        }

        private void SendConnectInd()
        {
            var pdu = new List<byte>();
            pdu.AddRange(AdvAccessAddress);
            pdu.Add(0x85);
            pdu.Add(0x22);
            pdu.AddRange(InitA);
            pdu.AddRange(advA);
            pdu.AddRange(DataAccessAddress);
            pdu.AddRange(new byte[] { 0x55, 0x55, 0x55 });
            pdu.Add(0x02);
            pdu.AddRange(new byte[] { 0x00, 0x00 });
            pdu.Add((byte)(IntervalUnits & 0xFF));
            pdu.Add((byte)(IntervalUnits >> 8));
            pdu.AddRange(new byte[] { 0x00, 0x00 });
            pdu.AddRange(new byte[] { 0xC8, 0x00 });
            pdu.AddRange(new byte[] { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F });
            pdu.Add(0x0D);
            Inject(pdu);
            txSn = 0;
            nesn = 0;
            inflight = null;
            this.Log(LogLevel.Info, "vphone: CONNECT_IND sent (interval {0} x 1.25 ms)", IntervalUnits);
        }

        private void SendDataPdu()
        {
            if(inflight == null)
            {
                if(controlQueue.Count > 0)
                {
                    inflight = controlQueue.Dequeue();
                }
                else if(dataQueue.Count > 0)
                {
                    inflight = dataQueue.Dequeue();
                }
                else
                {
                    inflight = new byte[0];
                }
            }
            var llid = inflight.Length == 0 ? 1 : inflight[0];
            var body = inflight.Length == 0 ? new byte[0] : inflight.Skip(1).ToArray();
            var more = (controlQueue.Count > 0 || dataQueue.Count > 0) && pdusThisEvent < MaxPdusPerEvent;
            var pdu = new List<byte>();
            pdu.AddRange(DataAccessAddress);
            pdu.Add((byte)(llid | (nesn << 2) | (txSn << 3) | (more ? 0x10 : 0)));
            pdu.Add((byte)body.Length);
            pdu.AddRange(body);
            Inject(pdu);
        }

        private void Inject(List<byte> pdu)
        {
            var arr = pdu.ToArray();
            var crc = radio.BuildFrameCrc(arr);
            radio.ReceiveFrame(arr.Concat(crc).ToArray(), null);
        }

        private void QueueControl(params byte[] body)
        {
            controlQueue.Enqueue(new byte[] { 3 }.Concat(body).ToArray());
        }

        private void QueueL2cap(ushort cid, byte[] sdu)
        {
            var frame = new List<byte> { (byte)sdu.Length, (byte)(sdu.Length >> 8), (byte)cid, (byte)(cid >> 8) };
            frame.AddRange(sdu);
            var first = true;
            for(var off = 0; off < frame.Count; off += maxTx)
            {
                var n = Math.Min(maxTx, frame.Count - off);
                var frag = new byte[n + 1];
                frag[0] = (byte)(first ? 2 : 1);
                frame.CopyTo(off, frag, 1, n);
                dataQueue.Enqueue(frag);
                first = false;
            }
        }

        private void HandlePeerPdu(int llid, byte[] payload)
        {
            if(llid == 3)
            {
                HandleControl(payload);
                return;
            }
            if(llid == 2)
            {
                rxL2cap.Clear();
            }
            rxL2cap.AddRange(payload);
            if(rxL2cap.Count < 4)
            {
                return;
            }
            var sduLen = rxL2cap[0] | (rxL2cap[1] << 8);
            if(rxL2cap.Count < sduLen + 4)
            {
                return;
            }
            var cid = rxL2cap[2] | (rxL2cap[3] << 8);
            var sdu = rxL2cap.Skip(4).Take(sduLen).ToArray();
            rxL2cap.Clear();
            switch(cid)
            {
            case 4:
                HandleAtt(sdu);
                break;
            case 5:
                HandleSignaling(sdu);
                break;
            case 6:
                if(!smpLogged)
                {
                    smpLogged = true;
                    this.Log(LogLevel.Info, "vphone: arm sent SMP {0} (ignored, no pairing)", Hex(sdu));
                }
                break;
            default:
                this.Log(LogLevel.Info, "vphone: L2CAP cid 0x{0:X} {1}", cid, Hex(sdu));
                break;
            }
        }

        private void HandleControl(byte[] p)
        {
            var op = p[0];
            switch(op)
            {
            case 0x0C:
                QueueControl(0x0C, 0x0C, 0x59, 0x00, 0x01, 0x00);
                break;
            case 0x0E:
            case 0x08:
                QueueControl(0x09, (byte)(UseDle ? 0x20 : 0), 0, 0, 0, 0, 0, 0, 0);
                break;
            case 0x14:
                if(UseDle)
                {
                    QueueControl(0x15, 251, 0, 0x48, 0x08, 251, 0, 0x48, 0x08);
                    maxTx = Math.Min(251, p[1] | (p[2] << 8));
                    this.Log(LogLevel.Info, "vphone: data length {0} (arm request)", maxTx);
                }
                else
                {
                    QueueControl(0x15, 27, 0, 0x48, 0x01, 27, 0, 0x48, 0x01);
                }
                break;
            case 0x12:
                QueueControl(0x13);
                break;
            case 0x0F:
                QueueControl(0x11, 0x0F, 0x1A);
                break;
            case 0x02:
                this.Log(LogLevel.Warning, "vphone: arm terminated the connection, reason 0x{0:X2}", p.Length > 1 ? p[1] : 0);
                break;
            case 0x15:
                maxTx = Math.Min(251, p[1] | (p[2] << 8));
                this.Log(LogLevel.Info, "vphone: data length {0} (arm response)", maxTx);
                break;
            case 0x07:
            case 0x09:
            case 0x0D:
            case 0x11:
                break;
            default:
                QueueControl(0x07, op);
                break;
            }
            this.Log(LogLevel.Debug, "vphone: LL control {0}", Hex(p));
        }

        private void HandleSignaling(byte[] s)
        {
            if(s.Length >= 4 && s[0] == 0x12)
            {
                QueueL2cap(5, new byte[] { 0x13, s[1], 0x02, 0x00, 0x01, 0x00 });
                this.Log(LogLevel.Info, "vphone: rejected connection parameter update {0}", Hex(s));
            }
            else if(s.Length >= 2 && s[0] != 0x13 && s[0] != 0x01)
            {
                QueueL2cap(5, new byte[] { 0x01, s[1], 0x02, 0x00, 0x00, 0x00 });
            }
        }

        // ---------------- ATT ----------------

        private void HandleAtt(byte[] a)
        {
            var op = a[0];
            if(AncsRequest(a))
            {
                return;
            }
            switch(op)
            {
            case 0x02:
                attMtu = Math.Min(DesiredMtu, a[1] | (a[2] << 8));
                QueueL2cap(4, new byte[] { 0x03, (byte)DesiredMtu, (byte)(DesiredMtu >> 8) });
                return;
            case 0x1B:
                LogNotification(a);
                return;
            case 0x1D:
                LogNotification(a);
                QueueL2cap(4, new byte[] { 0x1E });
                return;
            case 0x12:
                QueueL2cap(4, new byte[] { 0x13 });
                return;
            case 0x52:
                return;
            }
            if(op == 0x01 || (op & 1) == 1)
            {
                HandleAttResponse(a);
                return;
            }
            QueueL2cap(4, new byte[] { 0x01, op, 0x01, 0x00, 0x0A });
        }

        private void HandleAttResponse(byte[] a)
        {
            var op = a[0];
            if(!attPending)
            {
                this.Log(LogLevel.Debug, "vphone: unexpected ATT response {0}", Hex(a));
                return;
            }
            attPending = false;
            if(op == 0x03)
            {
                attMtu = Math.Min(DesiredMtu, a[1] | (a[2] << 8));
                this.Log(LogLevel.Info, "vphone: ATT MTU {0}", attMtu);
            }
            else if(op == 0x09)
            {
                var entry = a[1];
                for(var off = 2; off + entry <= a.Length; off += entry)
                {
                    var declHandle = a[off] | (a[off + 1] << 8);
                    var valueHandle = a[off + 3] | (a[off + 4] << 8);
                    var uuid = a.Skip(off + 5).Take(entry - 5).Reverse().ToArray();
                    var name = UuidName(uuid);
                    characteristics.Add(Tuple.Create(declHandle, valueHandle, Hex(uuid)));
                    if(name != null)
                    {
                        handles[name] = valueHandle;
                    }
                    discoverNext = declHandle + 1;
                }
            }
            else if(op == 0x01)
            {
                if(a.Length >= 5 && a[1] == 0x08)
                {
                    discoverDone = true;
                    foreach(var c in characteristics)
                    {
                        this.Log(LogLevel.Info, "vphone: char decl 0x{0:X4} value 0x{1:X4} uuid {2}", c.Item1, c.Item2, c.Item3);
                    }
                }
                else
                {
                    this.Log(LogLevel.Warning, "vphone: ATT error {0}", Hex(a));
                    bridgeLinkEvents.Add($"att-error {Hex(a)}");
                }
            }
        }

        // ---------------- the phone's ANCS ----------------

        private static readonly int AncsService = 0x0100, AncsNsDecl = 0x0101, AncsNs = 0x0102, AncsNsCcc = 0x0103, AncsCpDecl = 0x0104,
            AncsCp = 0x0105, AncsDsDecl = 0x0106, AncsDs = 0x0107, AncsDsCcc = 0x0108;
        private static readonly byte[] AncsServiceUuid = LittleEndianUuid("7905F431B5CE4E99A40F4B1E122D00D0");
        private static readonly byte[] AncsNsUuid = LittleEndianUuid("9FBF120D630142D98C5825E699A21DBD");
        private static readonly byte[] AncsCpUuid = LittleEndianUuid("69D1D8F345E149A898219BBDFDAAD9D9");
        private static readonly byte[] AncsDsUuid = LittleEndianUuid("22EAC6E924D64BB5BE44B36ACE7C7BFB");

        private static byte[] LittleEndianUuid(string hex)
        {
            return ParseHex(hex).Reverse().ToArray();
        }

        private void AncsCommand(string[] parts)
        {
            switch(parts[1])
            {
            case "on":
                ancsOn = true;
                break;
            case "off":
                ancsOn = false;
                break;
            case "error":
                ancsCpError = Convert.ToByte(parts[2], 16);
                break;
            case "ns":
            case "ds":
                var h = parts[1] == "ns" ? AncsNs : AncsDs;
                QueueL2cap(4, new byte[] { 0x1B, (byte)h, (byte)(h >> 8) }.Concat(ParseHex(parts[2])).ToArray());
                break;
            }
        }

        private void AttError(byte op, int handle, byte code)
        {
            QueueL2cap(4, new byte[] { 0x01, op, (byte)handle, (byte)(handle >> 8), code });
        }

        // What the arm asks as a GATT client. True when answered here; with ANCS off every query fails, as on a phone without it.
        private bool AncsRequest(byte[] a)
        {
            var op = a[0];
            if(op != 0x04 && op != 0x06 && op != 0x08 && op != 0x12)
            {
                return false;
            }
            if(op != 0x12)
            {
                bridgeLinkEvents.Add($"ancs request {op:X2}");
            }
            var first = a.Length >= 3 ? a[1] | (a[2] << 8) : 0;
            var last = a.Length >= 5 ? a[3] | (a[4] << 8) : 0;
            switch(op)
            {
            case 0x06:
                if(ancsOn && first <= AncsService && last >= AncsService && a.Length == 23 && (a[5] | (a[6] << 8)) == 0x2800
                    && a.Skip(7).SequenceEqual(AncsServiceUuid))
                {
                    QueueL2cap(4, new byte[] { 0x07, (byte)AncsService, (byte)(AncsService >> 8), (byte)AncsDsCcc, (byte)(AncsDsCcc >> 8) });
                }
                else
                {
                    AttError(op, first, 0x0A);
                }
                return true;
            case 0x08:
                {
                    var entries = new List<byte> { 0x09, 21 };
                    var chars = new[] { Tuple.Create(AncsNsDecl, AncsNs, (byte)0x10, AncsNsUuid),
                        Tuple.Create(AncsCpDecl, AncsCp, (byte)0x08, AncsCpUuid), Tuple.Create(AncsDsDecl, AncsDs, (byte)0x10, AncsDsUuid) };
                    if(ancsOn && a.Length == 7 && (a[5] | (a[6] << 8)) == 0x2803)
                    {
                        foreach(var c in chars)
                        {
                            if(c.Item1 >= first && c.Item1 <= last && entries.Count + 21 <= attMtu)
                            {
                                entries.AddRange(new[] { (byte)c.Item1, (byte)(c.Item1 >> 8), c.Item3, (byte)c.Item2, (byte)(c.Item2 >> 8) });
                                entries.AddRange(c.Item4);
                            }
                        }
                    }
                    if(entries.Count > 2)
                    {
                        QueueL2cap(4, entries.ToArray());
                    }
                    else
                    {
                        AttError(op, first, 0x0A);
                    }
                    return true;
                }
            case 0x04:
                {
                    // Runs of one UUID size, as the specification wants them.
                    var attrs = new[] { Tuple.Create(AncsService, 0x2800, (byte[])null), Tuple.Create(AncsNsDecl, 0x2803, (byte[])null),
                        Tuple.Create(AncsNs, 0, AncsNsUuid), Tuple.Create(AncsNsCcc, 0x2902, (byte[])null),
                        Tuple.Create(AncsCpDecl, 0x2803, (byte[])null), Tuple.Create(AncsCp, 0, AncsCpUuid),
                        Tuple.Create(AncsDsDecl, 0x2803, (byte[])null), Tuple.Create(AncsDs, 0, AncsDsUuid),
                        Tuple.Create(AncsDsCcc, 0x2902, (byte[])null) };
                    var inRange = ancsOn ? attrs.Where(x => x.Item1 >= first && x.Item1 <= last).ToArray() : new Tuple<int, int, byte[]>[0];
                    if(inRange.Length == 0)
                    {
                        AttError(op, first, 0x0A);
                        return true;
                    }
                    var wide = inRange[0].Item3 != null;
                    var rsp = new List<byte> { 0x05, (byte)(wide ? 2 : 1) };
                    foreach(var x in inRange)
                    {
                        var size = wide ? 18 : 4;
                        if((x.Item3 != null) != wide || rsp.Count + size > attMtu)
                        {
                            break;
                        }
                        rsp.Add((byte)x.Item1);
                        rsp.Add((byte)(x.Item1 >> 8));
                        if(wide)
                        {
                            rsp.AddRange(x.Item3);
                        }
                        else
                        {
                            rsp.Add((byte)x.Item2);
                            rsp.Add((byte)(x.Item2 >> 8));
                        }
                    }
                    QueueL2cap(4, rsp.ToArray());
                    return true;
                }
            default:
                if(first == AncsNsCcc || first == AncsDsCcc)
                {
                    bridgeLinkEvents.Add($"ancs ccc {(first == AncsNsCcc ? "ns" : "ds")} {(a.Length > 3 ? a[3] : 0):X2}");
                    QueueL2cap(4, new byte[] { 0x13 });
                    return true;
                }
                if(first == AncsCp)
                {
                    bridgeLinkEvents.Add($"ancs cp {Hex(a.Skip(3))}");
                    if(ancsCpError != 0)
                    {
                        AttError(op, first, ancsCpError);
                    }
                    else
                    {
                        QueueL2cap(4, new byte[] { 0x13 });
                    }
                    return true;
                }
                return false;
            }
        }

        private void LogNotification(byte[] a)
        {
            var handle = a[1] | (a[2] << 8);
            var value = a.Skip(3).ToArray();
            var text = $"t={Now():F1}ms h=0x{handle:X4} {Hex(value)}";
            notifications.Add(text);
            if(bridge != null)
            {
                bridgeNotifications.Add($"N {handle:X} {(long)(Now() * 1000)} {Hex(value)}");
            }
            lastNotifyHandle = handle;
            notifySeen++;
            this.Log(LogLevel.Info, "vphone: NOTIFY {0}", text.Length > 160 ? text.Substring(0, 160) + "..." : text);
        }

        private void SendAttRequest(params byte[] a)
        {
            attPending = true;
            QueueL2cap(4, a);
        }

        // ---------------- script ----------------

        private void OnPduAcked()
        {
        }

        private void Advance()
        {
            BridgeSync();
            // Optional G1 heartbeat (25 06 00 seq 04 seq) on NUS: the stock firmware closes a link that
            // carries no NUS traffic for ~32 s.
            if(HeartbeatMs > 0 && connected && handles.ContainsKey("NUS_RX") && Now() - lastHeartbeat >= HeartbeatMs)
            {
                lastHeartbeat = Now();
                var h = handles["NUS_RX"];
                var b = (byte)(heartbeatSeq++ & 0xFF);
                QueueL2cap(4, new byte[] { 0x52, (byte)h, (byte)(h >> 8), 0x25, 0x06, 0x00, b, 0x04, b });
            }
            while(stepIndex < steps.Count)
            {
                if(!RunStep(steps[stepIndex]))
                {
                    return;
                }
                stepIndex++;
                stepStarted = false;
            }
        }

        // Returns true when the step is complete.
        private bool RunStep(string[] s)
        {
            var cmd = s[0].ToLowerInvariant();
            var first = !stepStarted;
            stepStarted = true;
            if(first)
            {
                stepStartTime = Now();
            }
            switch(cmd)
            {
            case "note":
                this.Log(LogLevel.Info, "vphone: NOTE t={0:F1}ms {1}", Now(), string.Join(" ", s.Skip(1)));
                return true;
            case "wait":
                return Now() - stepStartTime >= double.Parse(s[1]);
            case "reconnect":
                if(first)
                {
                    lossesAtStart = linkLosses;
                }
                if(linkLosses > lossesAtStart && connected)
                {
                    this.Log(LogLevel.Info, "vphone: reconnected at t={0:F1}ms", Now());
                    return true;
                }
                if(Now() - stepStartTime > (s.Length > 1 ? double.Parse(s[1]) : 30000))
                {
                    this.Log(LogLevel.Warning, "vphone: no reconnect within timeout");
                    return true;
                }
                return false;
            case "mtu":
                if(first)
                {
                    var n = int.Parse(s[1]);
                    SendAttRequest(0x02, (byte)n, (byte)(n >> 8));
                }
                return !attPending;
            case "discover":
                if(first)
                {
                    discoverNext = 1;
                    discoverDone = false;
                }
                if(attPending)
                {
                    return false;
                }
                if(discoverDone || discoverNext > 0xFFFF)
                {
                    return true;
                }
                SendAttRequest(0x08, (byte)discoverNext, (byte)(discoverNext >> 8), 0xFF, 0xFF, 0x03, 0x28);
                return false;
            case "subscribe":
                if(first)
                {
                    var ccc = Resolve(s[1]) + 1;
                    SendAttRequest(0x12, (byte)ccc, (byte)(ccc >> 8), 0x01, 0x00);
                }
                return !attPending;
            case "wr":
                if(first)
                {
                    var hr = Resolve(s[1]);
                    SendAttRequest(new byte[] { 0x12, (byte)hr, (byte)(hr >> 8) }.Concat(ParseHex(s[2])).ToArray());
                }
                return !attPending;
            case "w":
            case "wn":
                if(first)
                {
                    var h = Resolve(s[1]);
                    var value = ParseHex(s[2]);
                    if(value.Length > attMtu - 3)
                    {
                        this.Log(LogLevel.Error, "vphone: write of {0} bytes exceeds MTU {1}", value.Length, attMtu);
                    }
                    QueueL2cap(4, new byte[] { 0x52, (byte)h, (byte)(h >> 8) }.Concat(value).ToArray());
                    notifyAtStart = notifySeen;
                    return false;
                }
                if(dataQueue.Count > 0 || inflight != null)
                {
                    return false;
                }
                if(cmd == "w")
                {
                    return true;
                }
                var timeout = s.Length > 3 ? double.Parse(s[3]) : 5000;
                if(notifySeen > notifyAtStart)
                {
                    return true;
                }
                if(Now() - stepStartTime > timeout)
                {
                    this.Log(LogLevel.Warning, "vphone: no notification within {0} ms after step {1}", timeout, stepIndex);
                    return true;
                }
                return false;
            default:
                this.Log(LogLevel.Error, "vphone: unknown script command {0}", cmd);
                return true;
            }
        }

        private int Resolve(string name)
        {
            if(handles.TryGetValue(name, out var h))
            {
                return h;
            }
            return Convert.ToInt32(name, 16);
        }

        private int Handle(string name)
        {
            return handles.TryGetValue(name, out var h) ? h : 0;
        }

        private static string UuidName(byte[] uuidBigEndian)
        {
            var s = Hex(uuidBigEndian).ToLowerInvariant();
            if(s == "6e400002b5a3f393e0a9e50e24dcca9e") return "NUS_RX";
            if(s == "6e400003b5a3f393e0a9e50e24dcca9e") return "NUS_TX";
            if(s == "da2e7828fbce4e01ae9e261174997c48") return "SMP";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4401") return "G1_CMD";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4402") return "G1_CTRL";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4403") return "G1_EVT";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4404") return "G1_PAIR";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4405") return "G1_FC";
            if(s == "f3da4d59277a4fc9a1ed41c9c22d4406") return "G1_ANCS";
            return null;
        }

        private double Now()
        {
            return machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds * 1000.0;
        }

        private static bool Same(byte[] a, byte[] b)
        {
            for(var i = 0; i < 4; i++)
            {
                if(a[i] != b[i]) return false;
            }
            return true;
        }

        private static string Hex(IEnumerable<byte> b)
        {
            var sb = new StringBuilder();
            foreach(var x in b)
            {
                sb.Append(x.ToString("X2"));
            }
            return sb.ToString();
        }

        private static byte[] ParseHex(string hex)
        {
            var b = new byte[hex.Length / 2];
            for(var i = 0; i < b.Length; i++)
            {
                b[i] = Convert.ToByte(hex.Substring(2 * i, 2), 16);
            }
            return b;
        }

        private readonly NRF5340_RADIO radio;
        private byte[] advA;
        private string lastAdv;
        private bool scanned;
        private bool connected;
        private int txSn, nesn;
        private bool peerMd;
        private byte[] inflight;
        private readonly Queue<byte[]> controlQueue = new Queue<byte[]>();
        private readonly Queue<byte[]> dataQueue = new Queue<byte[]>();
        private readonly List<byte> rxL2cap = new List<byte>();
        private int attMtu = 23;
        private int maxTx = MaxLlPayload;
        private double lastHeartbeat = -1e9;
        private int heartbeatSeq;
        private bool attPending;
        private bool ancsOn;
        private byte ancsCpError;
        private int discoverNext;
        private bool discoverDone;
        private readonly List<Tuple<int, int, string>> characteristics = new List<Tuple<int, int, string>>();
        private readonly Dictionary<string, int> handles = new Dictionary<string, int>();
        private readonly List<string[]> steps = new List<string[]>();
        private int stepIndex;
        private bool stepStarted;
        private double stepStartTime;
        private int notifySeen, notifyAtStart, lastNotifyHandle;
        private int linkLosses, lossesAtStart;
        private readonly List<string> notifications = new List<string>();
        private bool smpLogged;
        private double lastEventStart = -1e9;
        private int pdusThisEvent;
        private long events, txPdus, rxPdus, injected, peerResponses, naks;

        private string bridgePath;
        private Socket listener, bridge;
        private NetworkStream bridgeStream;
        private System.IO.StreamReader bridgeReader;
        private System.IO.StreamWriter bridgeWriter;
        private readonly HashSet<string> handlesSent = new HashSet<string>();
        private readonly List<string> bridgeNotifications = new List<string>();
        private readonly List<string> bridgeLinkEvents = new List<string>();
        private readonly LimitTimer tickTimer;
        private double silentUntil = -1;

        private const int DesiredMtu = 247;
        private const int MaxLlPayload = 27;
        private static readonly byte[] AdvAccessAddress = { 0xD6, 0xBE, 0x89, 0x8E };
        private static readonly byte[] DataAccessAddress = { 0x29, 0x41, 0x76, 0x71 };
        // Same public address as g1-fw's virtual central; the QSPI seed stores it as the bonded phone.
        private static readonly byte[] InitA = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
    }
}
