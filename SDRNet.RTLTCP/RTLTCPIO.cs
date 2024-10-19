using System.Net.Sockets;
using SDRSharp.Radio;

namespace SDRSharp.RTLTCP
{
    public enum RtlSdrTunerType
    {
        Unknown = 0,
        E4000,
        FC0012,
        FC0013,
        FC2580,
        R820T
    }

    public unsafe class RtlTcpIO(string host, int port) : IFrontendController, IDisposable
    {
        private const int DongleInfoLength = 12;
        private const uint DefaultFrequency = 100000000;
        private readonly static int _bufferSize = 16 * 1024;
        
        #region Native rtl_tcp Commands

        private const byte CMD_SET_FREQ = 0x1;
        private const byte CMD_SET_SAMPLE_RATE = 0x2;
        private const byte CMD_SET_TUNER_GAIN_MODE = 0x3;
        private const byte CMD_SET_GAIN = 0x4;
        private const byte CMD_SET_FREQ_COR = 0x5;
        private const byte CMD_SET_AGC_MODE = 0x8;
        private const byte CMD_SET_TUNER_GAIN_INDEX = 0xd;

        #endregion

        private static readonly float* _lutPtr;
        private static readonly UnsafeBuffer _lutBuffer = UnsafeBuffer.Create(256, sizeof(float));

        private long _frequency = DefaultFrequency;
        private double _sampleRate;
        private readonly string _host = host;
        private readonly int _port = port;
        private bool _useRtlAGC;
        private bool _useTunerAGC;
        private uint _tunerGainIndex;
        private uint _tunerGainCount;
        private uint _tunerType;
        private int _frequencyCorrection;
        private SamplesAvailableDelegate? _callback;
        private Thread? _sampleThread;
        private UnsafeBuffer? _iqBuffer;        
        private Complex* _iqBufferPtr;
        private Socket? _s;
        private readonly byte [] _cmdBuffer = new byte[5];
                            
        #region Public Properties

        public bool IsStreaming
        {
            get { return _sampleThread != null; }
        }

        public bool IsSoundCardBased
        {
            get { return false; }
        }

        public string SoundCardHint
        {
            get { return string.Empty; }
        }

        public RtlSdrTunerType TunerType
        {
            get { return (RtlSdrTunerType) _tunerType; }
        }

        public double Samplerate
        {
            get { return _sampleRate; }
            set
            {
                _sampleRate = value;
                SendCommand(CMD_SET_SAMPLE_RATE, (uint) _sampleRate);
            }
        }

        public long Frequency
        {
            get { return _frequency; }
            set
            {
                _frequency = value;
                SendCommand(CMD_SET_FREQ, (uint) _frequency);
            }
        }

        public int FrequencyCorrection
        {
            get { return _frequencyCorrection; }
            set
            {
                _frequencyCorrection = value;
                SendCommand(CMD_SET_FREQ_COR, _frequencyCorrection);
            }
        }

        public bool UseRtlAGC
        {
            get { return _useRtlAGC; }
            set
            {
                _useRtlAGC = value;
                SendCommand(CMD_SET_AGC_MODE, _useRtlAGC ? 1: 0);
            }
        }

        public bool UseTunerAGC
        {
            get { return _useTunerAGC; }
            set
            {
                _useTunerAGC = value;
                SendCommand(CMD_SET_TUNER_GAIN_MODE, _useTunerAGC ? 0: 1);
            }
        }

        public uint TunerGainIndex
        {
            get { return _tunerGainIndex; }
            set
            {
                _tunerGainIndex = value;
                SendCommand(CMD_SET_TUNER_GAIN_INDEX, (int)_tunerGainIndex);
            }
        }

        public uint TunerGainCount
        {
            get { return _tunerGainCount; }
        }

        #endregion

        static RtlTcpIO()
        {
            _lutPtr = (float*) _lutBuffer;

            const float scale = 1.0f / 127.5f;
            for (var i = 0; i < 256; i++)
            {
                _lutPtr[i] = (i - 127.5f) * scale;
            }
        }

        ~RtlTcpIO()
        {
            Dispose();
        }
                
        public void Dispose()
        {
            if (_iqBuffer != null)
            {
                _iqBuffer.Dispose();
                _iqBuffer = null;
                _iqBufferPtr = null;
            }
            GC.SuppressFinalize(this);
        }

        public void Open()
        {
        }

        public void Close()
        {
            if (_s != null)
            {
                _s.Close();
                _s = null;
            }
        }

        public void Start(SamplesAvailableDelegate callback)
        {
            _callback = callback;
            _s = new(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp)
            {
                NoDelay = true
            };
            _s.Connect(_host, _port);

            var dongleInfo = new byte[DongleInfoLength];

            var length = _s.Receive(dongleInfo, 0, DongleInfoLength, SocketFlags.None);
            if (length > 0)
            {
                ParseDongleInfo(dongleInfo);
            }
            
            SendCommand(CMD_SET_SAMPLE_RATE, (uint)_sampleRate);
            SendCommand(CMD_SET_FREQ_COR, _frequencyCorrection);
            SendCommand(CMD_SET_FREQ, (uint)_frequency);
            SendCommand(CMD_SET_AGC_MODE, (uint)(_useRtlAGC ? 1 : 0));
            SendCommand(CMD_SET_TUNER_GAIN_MODE, (uint)(_useTunerAGC ? 0 : 1));
            SendCommand(CMD_SET_TUNER_GAIN_INDEX, (_tunerGainIndex));

            _sampleThread = new Thread(RecieveSamples);            
            _sampleThread.Start();
        }

        public void Stop()
        {
            Close();
            if (_sampleThread != null)
            {
                _sampleThread.Join();
                _sampleThread = null;
            }
            _callback = null;           
        }

        #region Private Methods

        private void ParseDongleInfo(byte[] buffer)
        {
            if (buffer.Length < DongleInfoLength)
            {
                return;
            }

            if (buffer[0] != 'R' || buffer[1] != 'T' || buffer[2] != 'L' || buffer[3] != '0')
            {
                _tunerType = 0;
                _tunerGainCount = 0;
                return;
            }
            _tunerType = (uint)(buffer[4] << 24 | buffer[5] << 16 | buffer[6] << 8 | buffer[7]);
            _tunerGainCount = (uint)(buffer[8] << 24 | buffer[9] << 16 | buffer[10] << 8 | buffer[11]);                        
        }

        public bool SendCommand(byte cmd, byte[] val)
        {
            if (_s == null || val.Length < 4)
            {
                return false;
            }
            
            _cmdBuffer[0] = cmd;
            _cmdBuffer[1] = val[3]; //Network byte order
            _cmdBuffer[2] = val[2];
            _cmdBuffer[3] = val[1];
            _cmdBuffer[4] = val[0];
            try
            {
                _s.Send(_cmdBuffer);
            }
            catch
            {
                return false;
            }
            return true;
        }

        private void SendCommand(byte cmd, UInt32 val)
        {
            var valBytes = BitConverter.GetBytes(val);
            SendCommand(cmd, valBytes);
        }

        private void SendCommand(byte cmd, Int32 val)
        {
            var valBytes = BitConverter.GetBytes(val);
            SendCommand(cmd, valBytes);
        }

        #endregion

        #region Worker Thread

        private void RecieveSamples()
        {
            var recBuffer = new byte[_bufferSize];
            var recUnsafeBuffer = UnsafeBuffer.Create(recBuffer);
            var recPtr = (byte*) recUnsafeBuffer;
            _iqBuffer = UnsafeBuffer.Create(_bufferSize / 2, sizeof(Complex));
            _iqBufferPtr = (Complex*) _iqBuffer;
            var offs = 0;                        
            while (_s != null && _s.Connected)
            {
                try
                {
                    var bytesRec = _s.Receive(recBuffer, offs, _bufferSize - offs, SocketFlags.None);
                    var totalBytes = offs + bytesRec;
                    offs = totalBytes % 2; //Need to correctly handle the hypothetical case where we somehow get an odd number of bytes
                    ProcessSamples(recPtr, totalBytes - offs); //This might work.
                    if (offs == 1)
                    {
                        recPtr[0] = recPtr[totalBytes - 1];
                    }
                }
                catch
                {
                    Close();
                    break;
                }
            }            
        }

        private void ProcessSamples(byte* rawPtr, int len)
        {
            var sampleCount = len / 2;

            var ptr = _iqBufferPtr;
            for (var i = 0; i < sampleCount; i++)
            {
                ptr->Imag = _lutPtr[*rawPtr++];
                ptr->Real = _lutPtr[*rawPtr++];
                ptr++;
            }
            _callback?.Invoke(this, _iqBufferPtr, sampleCount);
        }

        #endregion
    }
}
