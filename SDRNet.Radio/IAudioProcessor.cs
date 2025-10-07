using System;

namespace SDRNet.Radio
{
    public unsafe interface IAudioProcessor
    {
        double SampleRate { set; }
        bool Bypass { get; set; }
        void Process(float* audioBuffer, int length);
    }
}
