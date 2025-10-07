namespace SDRNet.HackRfOne
{
    public class DeviceDisplay
    {
        public uint Index { get; private set; }
        public string Name { get; set; }

        public static DeviceDisplay[] GetActiveDevices()
        {
            var count = 1;
            var result = new DeviceDisplay[count];

            var name = NativeMethods.hackrf_board_id_name(1);
            result[0] = new DeviceDisplay { Index = 0, Name = name };
            /*
            for (var i = 0u; i < count; i++)
            {
                var name = NativeMethods.rtlsdr_get_device_name(i);
                result[i] = new DeviceDisplay { Index = i, Name = name };
            }
            */
            return result;
        }

        public override string ToString()
        {
            return Name;
        }
    }
}
