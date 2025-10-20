namespace SDRNet.Radio
{
    public unsafe delegate void SamplesAvailableDelegate(IFrontendController sender, Complex* data, int len);

    public interface IFrontendController
    {
        event SamplesAvailableDelegate SamplesAvailable;

        void Open();
        void Start();
        void Stop();
        void Close();
        bool IsSoundCardBased { get; }
        string SoundCardHint { get; }
        double Samplerate { get; set; }
        long Frequency { get; set; }
    }
}
