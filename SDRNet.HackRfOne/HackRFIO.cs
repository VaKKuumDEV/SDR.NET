using SDRNet.Radio;

namespace SDRNet.HackRfOne
{
    public unsafe class HackRFIO : IFrontendController, ITransmitter, IDisposable
    {
        private HackRFDevice? _hackRFDevice;
        private long _frequency = 105500000;
        private double _frequencyCorrection;

        public event Radio.SamplesAvailableDelegate SamplesAvailable;
        public event TxSamplesNeededDelegate? TxSamplesNeeded;

        ~HackRFIO()
        {
            Dispose();
        }

        public void Dispose()
        {
            Close();
            GC.SuppressFinalize(this);
        }

        public void SelectDevice(uint index)
        {
            Close();
            _hackRFDevice = new HackRFDevice();
            _hackRFDevice.SamplesAvailable += HackRFDevice_SamplesAvailable;
            _hackRFDevice.TxSamplesNeeded += HackRFDevice_TxSamplesNeeded;
            _hackRFDevice.Frequency = _frequency;
        }

        public HackRFDevice? Device
        {
            get { return _hackRFDevice; }
        }

        public void Open()
        {
            var devices = DeviceDisplay.GetActiveDevices();
            foreach (var device in devices)
            {
                try
                {
                    SelectDevice(device.Index);
                    return;
                }
                catch (ApplicationException)
                {
                    // Just ignore it
                }
            }
            if (devices.Length > 0)
            {
                throw new ApplicationException(devices.Length + " compatible devices have been found but are all busy");
            }
            throw new ApplicationException("No compatible devices found");
        }

        public void Close()
        {
            if (_hackRFDevice != null)
            {
                _hackRFDevice.SamplesAvailable -= HackRFDevice_SamplesAvailable;
                _hackRFDevice.TxSamplesNeeded -= HackRFDevice_TxSamplesNeeded;
                _hackRFDevice.Dispose();
                _hackRFDevice = null;
            }
        }

        public void Start()
        {
            if (_hackRFDevice == null)
            {
                throw new ApplicationException("No device selected");
            }
            
            try
            {
                _hackRFDevice.Start();
            }
            catch
            {
                Open();
                _hackRFDevice.Start();
            }
        }

        public void Stop()
        {
            _hackRFDevice?.Stop();
        }

        public bool IsSoundCardBased
        {
            get { return false; }
        }

        public string SoundCardHint
        {
            get { return string.Empty; }
        }

        public double Samplerate
        {
            get { return _hackRFDevice == null ? 0.0 : _hackRFDevice.SampleRate; }
            set { if (_hackRFDevice != null) _hackRFDevice.SampleRate = value; }
        }

        public long Frequency
        {

            get { return _frequency; }
            set
            {
                if (_hackRFDevice != null)
                {
                    _hackRFDevice.Frequency = (long)(value * (1 + _frequencyCorrection * 0.000001));
                    _frequency = value;
                }
            }
        }

        public double FrequencyCorrection
        {
            get { return _frequencyCorrection; }
            set
            {
                _frequencyCorrection = value;
                Frequency = _frequency;
            }
        }

        private void HackRFDevice_SamplesAvailable(object sender, SamplesAvailableEventArgs e) => SamplesAvailable?.Invoke(this, e.Buffer, e.Length);

        #region Transmit

        /// <summary>Признак активной передачи.</summary>
        public bool IsTransmitting
        {
            get { return _hackRFDevice?.IsTransmitting ?? false; }
        }

        /// <summary>Усиление тракта передачи TXVGA (0–47 дБ).</summary>
        public uint TxVGAGain
        {
            get { return _hackRFDevice?.TxVGAGain ?? 0; }
            set { if (_hackRFDevice != null) _hackRFDevice.TxVGAGain = value; }
        }

        /// <summary>Встроенный усилитель (front-end amplifier, +14 дБ).</summary>
        public bool EnableAmp
        {
            get { return _hackRFDevice?.EnableAmp ?? false; }
            set { if (_hackRFDevice != null) _hackRFDevice.EnableAmp = value; }
        }

        /// <summary>Запускает поток передачи. При необходимости открывает устройство.</summary>
        public void StartTransmit()
        {
            if (_hackRFDevice == null)
            {
                Open();
            }

            _hackRFDevice!.StartTransmit();
        }

        /// <summary>Останавливает поток передачи.</summary>
        public void StopTransmit()
        {
            _hackRFDevice?.StopTransmit();
        }

        private void HackRFDevice_TxSamplesNeeded(object sender, TxSamplesNeededEventArgs e) => TxSamplesNeeded?.Invoke(this, e);

        #endregion
    }
}
