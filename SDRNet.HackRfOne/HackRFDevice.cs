using SDRNet.Radio;
using System.Runtime.InteropServices;

namespace SDRNet.HackRfOne
{
    public unsafe sealed class HackRFDevice : IDisposable, ITransmitter
    {
        private const uint DefaultFrequency = 105500000;
        private const int DefaultSamplerate = 10000000;
        private const string DeviceName = "HackRF Jawbreaker";

        /// <summary>Максимальное усиление тракта передачи HackRF One, дБ.</summary>
        public const uint MaxTxVGAGain = 47;

        private static readonly float* _lutPtr;
        private static readonly UnsafeBuffer _lutBuffer = UnsafeBuffer.Create(256, sizeof(float));

        private nint _dev;
        private long _centerFrequency = DefaultFrequency;
        private double _sampleRate = DefaultSamplerate;
        private uint _lnaGain;
        private uint _vgaGain;
        private uint _txVgaGain;
        private bool _amp;

        private GCHandle _gcHandle;
        private UnsafeBuffer? _iqBuffer;
        private Complex* _iqPtr;
        private bool _isStreaming;
        private readonly SamplesAvailableEventArgs _eventArgs = new();
        private static readonly hackrf_sample_block_cb_fn _HackRFCallback = HackRFSamplesAvailable;
        private static readonly uint _readLength = (uint)16 * 1024;

        private UnsafeBuffer? _txIqBuffer;
        private Complex* _txIqPtr;
        private bool _isTxStreaming;
        private readonly TxSamplesNeededEventArgs _txEventArgs = new();
        private static readonly hackrf_sample_block_cb_fn _HackRFTxCallback = HackRFTxSamplesAvailable;

        static HackRFDevice()
        {
            _lutPtr = (float*)_lutBuffer;

            const float scale = 1.0f / 127.0f;
            for (var i = 0; i < 256; i++)
            {
                _lutPtr[i] = (i - 128) * scale;
            }
        }

        public HackRFDevice()
        {
            var r = NativeMethods.hackrf_init();
            if (r != 0)
            {
                throw new ApplicationException("Cannot init HackRF device. Is the device locked somewhere?");
            }

            r = NativeMethods.hackrf_open(out _dev);
            if (r != 0)
            {
                throw new ApplicationException("Cannot open HackRF device. Is the device locked somewhere?");
            }

            _gcHandle = GCHandle.Alloc(this);
        }

        ~HackRFDevice()
        {
            Dispose();
        }

        public void Dispose()
        {
            Stop();
            NativeMethods.hackrf_close(_dev);
            NativeMethods.hackrf_exit();
            if (_gcHandle.IsAllocated)
            {
                _gcHandle.Free();
            }
            _txIqBuffer?.Dispose();
            _txIqBuffer = null;
            _dev = nint.Zero;
            GC.SuppressFinalize(this);
        }

        public event SamplesAvailableDelegate? SamplesAvailable;

        public void Start()
        {
            if (_isStreaming)
            {
                throw new ApplicationException("Start() Already running");
            }

            if (_isTxStreaming)
            {
                throw new ApplicationException("HackRF is half-duplex: stop transmitting before receiving");
            }

            var r = NativeMethods.hackrf_set_sample_rate(_dev, _sampleRate);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_sample_rate_set() error");
            }

            r = NativeMethods.hackrf_set_amp_enable(_dev, (byte)(_amp ? 1 : 0));
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_amp_enable() error");
            }

            r = NativeMethods.hackrf_set_lna_gain(_dev, _lnaGain);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_lna_gain() error");
            }

            r = NativeMethods.hackrf_set_vga_gain(_dev, _vgaGain);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_vga_gain() error");
            }

            var baseband_filter_bw_hz = NativeMethods.hackrf_compute_baseband_filter_bw_round_down_lt((uint)_sampleRate);
            r = NativeMethods.hackrf_set_baseband_filter_bandwidth(_dev, baseband_filter_bw_hz);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_baseband_filter_bandwidth_set() error");
            }

            r = NativeMethods.hackrf_set_freq(_dev, _centerFrequency);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_freq() error");
            }

            r = NativeMethods.hackrf_start_rx(_dev, _HackRFCallback, (nint)_gcHandle);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_start_rx() error");
            }

            r = NativeMethods.hackrf_is_streaming(_dev);
            if (r != 1)
            {
                throw new ApplicationException("hackrf_is_streaming() Error");
            }

            _isStreaming = true;
        }

        public void Stop()
        {
            if (_isStreaming)
            {
                NativeMethods.hackrf_stop_rx(_dev);
                _isStreaming = false;
            }

            StopTransmit();
        }

        public uint Index
        {
            get { return 0; }
        }

        public string Name
        {
            get { return DeviceName; }
        }

        public uint LNAGain
        {
            get { return _lnaGain; }
            set
            {
                _lnaGain = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_lna_gain(_dev, _lnaGain);
                }
            }
        }

        public uint VGAGain
        {
            get { return _vgaGain; }
            set
            {
                _vgaGain = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_vga_gain(_dev, _vgaGain);

                }
            }
        }

        /// <summary>
        /// Усиление тракта передачи TXVGA в дБ. Значения ограничиваются диапазоном
        /// 0–47 дБ и квантуются драйвером шагом 1 дБ.
        /// </summary>
        public uint TxVGAGain
        {
            get { return _txVgaGain; }
            set
            {
                if (value > MaxTxVGAGain) value = MaxTxVGAGain;
                _txVgaGain = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_txvga_gain(_dev, _txVgaGain);
                }
            }
        }

        public bool EnableAmp
        {
            get { return _amp; }
            set
            {
                _amp = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_amp_enable(_dev, (byte)(_amp ? 1 : 0));
                }
            }
        }

        public double SampleRate
        {
            get { return _sampleRate; }
            set
            {
                _sampleRate = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_sample_rate(_dev, _sampleRate);
                }
            }
        }

        public long Frequency
        {
            get { return _centerFrequency; }
            set
            {
                _centerFrequency = value;
                if (_dev != nint.Zero)
                {
                    NativeMethods.hackrf_set_freq(_dev, _centerFrequency);
                }
            }
        }

        public bool IsStreaming
        {
            get { return _isStreaming || _isTxStreaming; }
        }

        /// <summary>Признак активного приёма.</summary>
        public bool IsReceiving
        {
            get { return _isStreaming; }
        }

        /// <summary>Признак активной передачи.</summary>
        public bool IsTransmitting
        {
            get { return _isTxStreaming; }
        }

        #region Streaming methods

        private void ComplexSamplesAvailable(Complex* buffer, int length)
        {
            if (SamplesAvailable != null)
            {
                _eventArgs.Buffer = buffer;
                _eventArgs.Length = length;
                SamplesAvailable(this, _eventArgs);
            }
        }

        private static int HackRFSamplesAvailable(hackrf_transfer* ptr)
        {
            byte* buf = ptr->buffer;
            int len = ptr->buffer_length;
            nint ctx = ptr->rx_ctx;

            var gcHandle = GCHandle.FromIntPtr(ctx);
            if (!gcHandle.IsAllocated) return -1;
            var instance = (HackRFDevice?)gcHandle.Target;
            if (instance == null) return -1;

            var sampleCount = len / 2;
            if (instance._iqBuffer == null || instance._iqBuffer.Length != sampleCount)
            {
                instance._iqBuffer = UnsafeBuffer.Create(sampleCount, sizeof(Complex));
                instance._iqPtr = (Complex*)instance._iqBuffer;
            }

            var ptrIq = instance._iqPtr;
            for (var i = 0; i < sampleCount; i++)
            {
                ptrIq->Imag = _lutPtr[*buf++];
                ptrIq->Real = _lutPtr[*buf++];
                ptrIq++;
            }

            instance.ComplexSamplesAvailable(instance._iqPtr, instance._iqBuffer.Length);
            return 0;
        }

        #endregion

        #region Transmit methods

        /// <summary>
        /// Возникает, когда драйверу нужен очередной блок отсчётов для передачи.
        /// Подписчик обязан заполнить предоставленный буфер.
        /// </summary>
        public event TxSamplesNeededDelegate? TxSamplesNeeded;

        /// <summary>Запускает поток передачи. HackRF One полудуплексный, поэтому приём должен быть остановлен.</summary>
        public void StartTransmit()
        {
            if (_isTxStreaming)
            {
                throw new ApplicationException("StartTransmit() Already running");
            }

            if (_isStreaming)
            {
                throw new ApplicationException("HackRF is half-duplex: stop receiving before transmitting");
            }

            ApplyTransmitParameters();

            var r = NativeMethods.hackrf_start_tx(_dev, _HackRFTxCallback, (nint)_gcHandle);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_start_tx() error");
            }

            r = NativeMethods.hackrf_is_streaming(_dev);
            if (r != 1)
            {
                throw new ApplicationException("hackrf_is_streaming() Error");
            }

            _isTxStreaming = true;
        }

        /// <summary>Останавливает поток передачи.</summary>
        public void StopTransmit()
        {
            if (!_isTxStreaming)
            {
                return;
            }

            NativeMethods.hackrf_stop_tx(_dev);
            _isTxStreaming = false;
        }

        /// <summary>
        /// Применяет к устройству параметры, общие для приёма и передачи:
        /// частоту дискретизации, частоту настройки, полосу фильтра и усилитель.
        /// </summary>
        private void ApplyTransmitParameters()
        {
            var r = NativeMethods.hackrf_set_sample_rate(_dev, _sampleRate);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_sample_rate_set() error");
            }

            r = NativeMethods.hackrf_set_amp_enable(_dev, (byte)(_amp ? 1 : 0));
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_amp_enable() error");
            }

            var baseband_filter_bw_hz = NativeMethods.hackrf_compute_baseband_filter_bw_round_down_lt((uint)_sampleRate);
            r = NativeMethods.hackrf_set_baseband_filter_bandwidth(_dev, baseband_filter_bw_hz);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_baseband_filter_bandwidth_set() error");
            }

            r = NativeMethods.hackrf_set_freq(_dev, _centerFrequency);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_freq() error");
            }

            r = NativeMethods.hackrf_set_txvga_gain(_dev, _txVgaGain);
            if (r != 0)
            {
                throw new ApplicationException("hackrf_set_txvga_gain() error");
            }
        }

        private void ComplexTxSamplesNeeded(Complex* buffer, int length)
        {
            // Зануляем буфер, чтобы при неполном заполнении подписчиком
            // в эфир не ушли устаревшие данные от предыдущего блока.
            SignalGenerator.Silence(buffer, length);

            if (TxSamplesNeeded != null)
            {
                _txEventArgs.Buffer = buffer;
                _txEventArgs.Length = length;
                TxSamplesNeeded(this, _txEventArgs);
            }
        }

        private static int HackRFTxSamplesAvailable(hackrf_transfer* ptr)
        {
            nint ctx = ptr->tx_ctx;

            var gcHandle = GCHandle.FromIntPtr(ctx);
            if (!gcHandle.IsAllocated) return -1;
            var instance = (HackRFDevice?)gcHandle.Target;
            if (instance == null) return -1;

            var sampleCount = ptr->buffer_length / 2;
            if (instance._txIqBuffer == null || instance._txIqBuffer.Length != sampleCount)
            {
                instance._txIqBuffer?.Dispose();
                instance._txIqBuffer = UnsafeBuffer.Create(sampleCount, sizeof(Complex));
                instance._txIqPtr = (Complex*)instance._txIqBuffer;
            }

            instance.ComplexTxSamplesNeeded(instance._txIqPtr, sampleCount);
            SampleConverter.ComplexToInt8(instance._txIqPtr, sampleCount, ptr->buffer);
            ptr->valid_length = ptr->buffer_length;

            return 0;
        }

        #endregion
    }

    public delegate void SamplesAvailableDelegate(object sender, SamplesAvailableEventArgs e);

    public unsafe sealed class SamplesAvailableEventArgs : EventArgs
    {
        public int Length { get; set; }
        public Complex* Buffer { get; set; }
    }
}
