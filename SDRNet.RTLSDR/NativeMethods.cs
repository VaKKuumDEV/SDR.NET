using System;
using System.Runtime.InteropServices;
using System.Text;

namespace SDRNet.RTLSDR
{
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public unsafe delegate void RtlSdrReadAsyncDelegate(byte* buf, uint len, nint ctx);

    public enum RtlSdrTunerType
    {
        Unknown = 0,
        E4000,
        FC0012,
        FC0013,
        FC2580,
        R820T
    }

    public class NativeMethods
    {
        private const string LibRtlSdr = "librtlsdr";

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_device_count", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint rtlsdr_get_device_count();

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_device_name", CallingConvention = CallingConvention.Cdecl)]
        private static extern nint rtlsdr_get_device_name_native(uint index);

        public static string rtlsdr_get_device_name(uint index)
        {
            var strptr = rtlsdr_get_device_name_native(index);
            return Marshal.PtrToStringAnsi(strptr);
        }

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_device_usb_strings", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_device_usb_strings(uint index, StringBuilder manufact, StringBuilder product, StringBuilder serial);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_open", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_open(out nint dev, uint index);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_close", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_close(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_xtal_freq", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_xtal_freq(nint dev, uint rtlFreq, uint tunerFreq);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_xtal_freq", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_xtal_freq(nint dev, out uint rtlFreq, out uint tunerFreq);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_usb_strings", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_usb_strings(nint dev, StringBuilder manufact, StringBuilder product, StringBuilder serial);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_center_freq", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_center_freq(nint dev, uint freq);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_center_freq", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint rtlsdr_get_center_freq(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_freq_correction", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_freq_correction(nint dev, int ppm);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_freq_correction", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_freq_correction(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_tuner_gains", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_tuner_gains(nint dev, [In, Out] int[] gains);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_tuner_type", CallingConvention = CallingConvention.Cdecl)]
        public static extern RtlSdrTunerType rtlsdr_get_tuner_type(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_tuner_gain", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_tuner_gain(nint dev, int gain);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_tuner_gain", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_get_tuner_gain(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_tuner_gain_mode", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_tuner_gain_mode(nint dev, int manual);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_agc_mode", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_agc_mode(nint dev, int on);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_direct_sampling", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_direct_sampling(nint dev, int on);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_offset_tuning", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_offset_tuning(nint dev, int on);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_sample_rate", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_sample_rate(nint dev, uint rate);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_get_sample_rate", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint rtlsdr_get_sample_rate(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_set_testmode", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_set_testmode(nint dev, int on);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_reset_buffer", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_reset_buffer(nint dev);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_read_sync", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_read_sync(nint dev, nint buf, int len, out int nRead);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_wait_async", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_wait_async(nint dev, RtlSdrReadAsyncDelegate cb, nint ctx);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_read_async", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_read_async(nint dev, RtlSdrReadAsyncDelegate cb, nint ctx, uint bufNum, uint bufLen);

        [DllImport(LibRtlSdr, EntryPoint = "rtlsdr_cancel_async", CallingConvention = CallingConvention.Cdecl)]
        public static extern int rtlsdr_cancel_async(nint dev);
    }
}
