namespace SDRNet.Radio
{
    public unsafe interface IIQObserver
    {
        void IQSamplesAvailable(Complex* buffer, int length);
        double SampleRate { set; }
        bool Enabled { get; }
    }
}
