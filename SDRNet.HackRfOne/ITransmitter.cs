using SDRNet.Radio;

namespace SDRNet.HackRfOne
{
    /// <summary>
    /// Делегат, вызываемый приёмником передачи, когда драйверу нужен очередной
    /// блок отсчётов. Подписчик обязан заполнить буфер <see cref="TxSamplesNeededEventArgs.Buffer"/>
    /// количеством отсчётов, указанным в <see cref="TxSamplesNeededEventArgs.Length"/>.
    /// </summary>
    public unsafe delegate void TxSamplesNeededDelegate(object sender, TxSamplesNeededEventArgs e);

    /// <summary>Аргументы события <see cref="ITransmitter.TxSamplesNeeded"/>.</summary>
    public unsafe sealed class TxSamplesNeededEventArgs : EventArgs
    {
        /// <summary>Количество комплексных отсчётов, которые нужно записать в буфер.</summary>
        public int Length { get; set; }

        /// <summary>Буфер комплексных отсчётов, который заполняет подписчик.</summary>
        public Complex* Buffer { get; set; }
    }

    /// <summary>
    /// Контракт устройства, способного передавать сигнал (TX).
    /// HackRF One — полудуплексное устройство, поэтому передача и приём
    /// одновременно невозможны.
    /// </summary>
    public interface ITransmitter
    {
        /// <summary>
        /// Возникает, когда драйверу нужен очередной блок отсчётов для передачи.
        /// </summary>
        event TxSamplesNeededDelegate TxSamplesNeeded;

        /// <summary>Признак активной передачи.</summary>
        bool IsTransmitting { get; }

        /// <summary>
        /// Усиление тракта передачи TXVGA в дБ. Допустимый диапазон — 0–47 дБ,
        /// значения квантуются шагом 1 дБ.
        /// </summary>
        uint TxVGAGain { get; set; }

        /// <summary>
        /// Встроенный усилитель (front-end amplifier, +14 дБ).
        /// Не рекомендуется использовать без антенны или аттенюатора.
        /// </summary>
        bool EnableAmp { get; set; }

        /// <summary>Запускает поток передачи.</summary>
        void StartTransmit();

        /// <summary>Останавливает поток передачи.</summary>
        void StopTransmit();
    }
}
