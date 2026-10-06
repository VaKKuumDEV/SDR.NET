using SDRNet.Radio;

namespace SDRNet.HackRfOne
{
    /// <summary>
    /// Генераторы тестовых и служебных сигналов для тракта передачи HackRF One.
    /// Все методы работают с буферами комплексных отсчётов в диапазоне [-1; 1].
    /// </summary>
    /// <remarks>
    /// Методы для периодических сигналов возвращают фазу, на которой завершилась
    /// генерация. Это позволяет сохранять непрерывность фазы между вызовами при
    /// потоковой передаче: передавайте возвращённое значение в следующий вызов.
    /// </remarks>
    public static unsafe class SignalGenerator
    {
        private const double TwoPi = 2.0 * Math.PI;

        /// <summary>
        /// Заполняет буфер комплексным гармоническим сигналом (тон, комплексная экспонента).
        /// </summary>
        /// <returns>Фаза, следующая за последним сгенерированным отсчётом.</returns>
        public static double GenerateTone(Complex* buffer, int length, double sampleRate, double frequency, float amplitude = 1.0f, double phase = 0.0)
        {
            var phaseStep = TwoPi * frequency / sampleRate;
            for (var i = 0; i < length; i++)
            {
                buffer[i].Real = (float)(Math.Cos(phase) * amplitude);
                buffer[i].Imag = (float)(Math.Sin(phase) * amplitude);
                phase += phaseStep;
            }
            return WrapPhase(phase);
        }

        /// <summary>
        /// Заполняет буфер вещественным гармоническим сигналом (например, для звуковой карты).
        /// </summary>
        /// <returns>Фаза, следующая за последним сгенерированным отсчётом.</returns>
        public static double GenerateTone(float* buffer, int length, double sampleRate, double frequency, float amplitude = 1.0f, double phase = 0.0)
        {
            var phaseStep = TwoPi * frequency / sampleRate;
            for (var i = 0; i < length; i++)
            {
                buffer[i] = (float)(Math.Sin(phase) * amplitude);
                phase += phaseStep;
            }
            return WrapPhase(phase);
        }

        /// <summary>Заполняет буфер одинаковыми комплексными отсчётами (постоянный сигнал / DC).</summary>
        public static void Fill(Complex* buffer, int length, Complex value)
        {
            for (var i = 0; i < length; i++)
            {
                buffer[i] = value;
            }
        }

        /// <summary>Заполняет буфер нулями (тишина в эфире).</summary>
        public static void Silence(Complex* buffer, int length)
        {
            for (var i = 0; i < length; i++)
            {
                buffer[i] = default;
            }
        }

        /// <summary>
        /// Генерирует комплексный белый шум равномерного распределения.
        /// </summary>
        public static void GenerateNoise(Complex* buffer, int length, float amplitude, Random? random = null)
        {
            random ??= Random.Shared;
            for (var i = 0; i < length; i++)
            {
                buffer[i].Real = (float)(random.NextDouble() * 2.0 - 1.0) * amplitude;
                buffer[i].Imag = (float)(random.NextDouble() * 2.0 - 1.0) * amplitude;
            }
        }

        /// <summary>
        /// Домножает буфер на комплексную экспоненту, сдвигая спектр сигнала
        /// на заданную частоту (цифровое преобразование частоты).
        /// </summary>
        /// <returns>Фаза, следующая за последним обработанным отсчётом.</returns>
        public static double ShiftFrequency(Complex* buffer, int length, double sampleRate, double shiftFrequency, double phase = 0.0)
        {
            var phaseStep = TwoPi * shiftFrequency / sampleRate;
            for (var i = 0; i < length; i++)
            {
                var rotation = Complex.FromAngle(phase);
                buffer[i] *= rotation;
                phase += phaseStep;
            }
            return WrapPhase(phase);
        }

        /// <summary>
        /// Применяет к началу и концу буфера сглаживающее окно (приподнятый косинус).
        /// Позволяет убрать скачки амплитуды при старте и остановке передачи и тем
        /// самым снизить внеполосное излучение.
        /// </summary>
        /// <param name="buffer">Буфер комплексных отсчётов.</param>
        /// <param name="length">Количество отсчётов.</param>
        /// <param name="rampSamples">Длительность фронта/спада в отсчётах.</param>
        public static void ApplyRamp(Complex* buffer, int length, int rampSamples)
        {
            if (rampSamples <= 0 || length == 0) return;

            var ramp = Math.Min(rampSamples, length / 2);
            if (ramp <= 0) return;

            for (var i = 0; i < ramp; i++)
            {
                var gain = 0.5f * (1.0f - MathF.Cos((float)Math.PI * i / ramp));
                buffer[i] *= gain;
                buffer[length - 1 - i] *= gain;
            }
        }

        /// <summary>Применяет постоянный коэффициент усиления к буферу отсчётов.</summary>
        public static void ApplyGain(Complex* buffer, int length, float gain)
            => SampleConverter.Scale(buffer, length, gain);

        /// <summary>Приводит фазу к диапазону [0; 2π), сохраняя точность при длительной генерации.</summary>
        private static double WrapPhase(double phase)
        {
            phase %= TwoPi;
            if (phase < 0.0) phase += TwoPi;
            return phase;
        }
    }
}
