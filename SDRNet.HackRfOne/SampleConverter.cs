using SDRNet.Radio;

namespace SDRNet.HackRfOne
{
    /// <summary>
    /// Методы преобразования отсчётов между форматами, которые используются
    /// HackRF One и библиотекой <c>SDRNet.Radio</c>.
    /// </summary>
    /// <remarks>
    /// HackRF One работает со знаковым 8-битным форматом (int8). Для каждого
    /// комплексного отсчёта в буфере хранится два байта, которые по соглашению
    /// данной библиотеки идут в порядке <c>[Q][I]</c>: первый байт — мнимая часть,
    /// второй — действительная (так же их читает приёмный тракт <c>HackRFDevice</c>).
    /// Значение байта <c>0</c> соответствует <c>-1.0</c>, <c>128</c> — нулю,
    /// <c>255</c> — примерно <c>+1.0</c>.
    /// </remarks>
    public static unsafe class SampleConverter
    {
        /// <summary>Масштаб для перевода знакового байта HackRF в диапазон [-1; 1].</summary>
        public const float Int8Scale = 1.0f / 127.0f;

        /// <summary>Масштаб для перевода 16-битного отсчёта в диапазон [-1; 1].</summary>
        public const float Int16Scale = 1.0f / 32767.0f;

        #region int8 (HackRF) <-> Complex

        /// <summary>
        /// Преобразует буфер HackRF (int8, чередование Q/I) в массив комплексных отсчётов.
        /// </summary>
        /// <param name="source">Исходный буфер, содержащий <c>2 * sampleCount</c> байт.</param>
        /// <param name="sampleCount">Количество комплексных отсчётов.</param>
        /// <param name="destination">Приёмный буфер комплексных отсчётов.</param>
        public static void Int8ToComplex(byte* source, int sampleCount, Complex* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                var imag = (source[0] - 128) * Int8Scale;
                var real = (source[1] - 128) * Int8Scale;
                source += 2;

                destination->Imag = imag;
                destination->Real = real;
                destination++;
            }
        }

        /// <summary>
        /// Преобразует комплексные отсчёты в буфер HackRF (int8, чередование Q/I).
        /// Значения ограничиваются диапазоном [-1; 1], чтобы избежать переполнения.
        /// </summary>
        /// <param name="source">Исходные комплексные отсчёты.</param>
        /// <param name="sampleCount">Количество комплексных отсчётов.</param>
        /// <param name="destination">Приёмный буфер, вмещающий <c>2 * sampleCount</c> байт.</param>
        public static void ComplexToInt8(Complex* source, int sampleCount, byte* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                destination[0] = FloatToInt8(source->Imag);
                destination[1] = FloatToInt8(source->Real);
                destination += 2;
                source++;
            }
        }

        /// <summary>Преобразует буфер HackRF (int8) в массив комплексных отсчётов.</summary>
        public static Complex[] Int8ToComplex(byte[] data)
        {
            ArgumentNullException.ThrowIfNull(data);

            var result = new Complex[data.Length / 2];
            for (var i = 0; i < result.Length; i++)
            {
                result[i].Imag = (data[i * 2] - 128) * Int8Scale;
                result[i].Real = (data[i * 2 + 1] - 128) * Int8Scale;
            }
            return result;
        }

        /// <summary>Преобразует массив комплексных отсчётов в буфер HackRF (int8).</summary>
        public static byte[] ComplexToInt8(Complex[] samples)
        {
            ArgumentNullException.ThrowIfNull(samples);

            var result = new byte[samples.Length * 2];
            for (var i = 0; i < samples.Length; i++)
            {
                result[i * 2] = FloatToInt8(samples[i].Imag);
                result[i * 2 + 1] = FloatToInt8(samples[i].Real);
            }
            return result;
        }

        /// <summary>Преобразует буфер HackRF (int8) в комплексные отсчёты.</summary>
        public static void Int8ToComplex(ReadOnlySpan<byte> source, Span<Complex> destination)
        {
            if (destination.Length < source.Length / 2)
            {
                throw new ArgumentException("Приёмный буфер слишком мал.", nameof(destination));
            }

            for (var i = 0; i < destination.Length && i * 2 + 1 < source.Length; i++)
            {
                destination[i] = new Complex(
                    (source[i * 2 + 1] - 128) * Int8Scale,
                    (source[i * 2] - 128) * Int8Scale);
            }
        }

        /// <summary>Преобразует комплексные отсчёты в буфер HackRF (int8).</summary>
        public static void ComplexToInt8(ReadOnlySpan<Complex> source, Span<byte> destination)
        {
            if (destination.Length < source.Length * 2)
            {
                throw new ArgumentException("Приёмный буфер слишком мал.", nameof(destination));
            }

            for (var i = 0; i < source.Length; i++)
            {
                destination[i * 2] = FloatToInt8(source[i].Imag);
                destination[i * 2 + 1] = FloatToInt8(source[i].Real);
            }
        }

        /// <summary>
        /// Преобразует одно вещественное значение в знаковый байт HackRF со
        /// ограничением по амплитуде и кодированием со смещением 128.
        /// </summary>
        public static byte FloatToInt8(float value)
        {
            var scaled = (int)(value * 127.0f + (value >= 0.0f ? 0.5f : -0.5f));
            if (scaled > 127) scaled = 127;
            else if (scaled < -128) scaled = -128;
            return (byte)(scaled + 128);
        }

        /// <summary>Преобразует знаковый байт HackRF обратно в вещественное значение.</summary>
        public static float Int8ToFloat(byte value) => (value - 128) * Int8Scale;

        #endregion

        #region int16 (звуковые карты / WAV) <-> Complex

        /// <summary>
        /// Преобразует чередующиеся 16-битные отсчёты (I/Q) в комплексные.
        /// </summary>
        public static void Int16ToComplex(short* source, int sampleCount, Complex* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                destination->Real = source[0] * Int16Scale;
                destination->Imag = source[1] * Int16Scale;
                source += 2;
                destination++;
            }
        }

        /// <summary>
        /// Преобразует комплексные отсчёты в чередующиеся 16-битные (I/Q).
        /// </summary>
        public static void ComplexToInt16(Complex* source, int sampleCount, short* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                destination[0] = FloatToInt16(source->Real);
                destination[1] = FloatToInt16(source->Imag);
                destination += 2;
                source++;
            }
        }

        /// <summary>Преобразует один отсчёт в знаковое 16-битное значение с ограничением.</summary>
        public static short FloatToInt16(float value)
        {
            var scaled = (int)(value * 32767.0f + (value >= 0.0f ? 0.5f : -0.5f));
            if (scaled > 32767) scaled = 32767;
            else if (scaled < -32768) scaled = -32768;
            return (short)scaled;
        }

        /// <summary>Преобразует знаковое 16-битное значение в диапазон [-1; 1].</summary>
        public static float Int16ToFloat(short value) => value * Int16Scale;

        #endregion

        #region float <-> Complex

        /// <summary>
        /// Преобразует чередующиеся вещественные отсчёты (I/Q) в комплексные.
        /// </summary>
        public static void FloatToComplex(float* source, int sampleCount, Complex* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                destination->Real = source[0];
                destination->Imag = source[1];
                source += 2;
                destination++;
            }
        }

        /// <summary>
        /// Преобразует комплексные отсчёты в чередующиеся вещественные (I/Q).
        /// </summary>
        public static void ComplexToFloat(Complex* source, int sampleCount, float* destination)
        {
            for (var i = 0; i < sampleCount; i++)
            {
                destination[0] = source->Real;
                destination[1] = source->Imag;
                destination += 2;
                source++;
            }
        }

        #endregion

        #region Операции над амплитудой

        /// <summary>Возвращает максимальный модуль среди отсчётов буфера.</summary>
        public static float MaxModulus(Complex* source, int count)
        {
            var max = 0.0f;
            for (var i = 0; i < count; i++)
            {
                var modulus = source[i].Modulus();
                if (modulus > max) max = modulus;
            }
            return max;
        }

        /// <summary>
        /// Нормирует буфер так, чтобы максимальный модуль был равен 1.
        /// Буфер из нулей остаётся без изменений.
        /// </summary>
        public static void Normalize(Complex* source, int count)
        {
            var max = MaxModulus(source, count);
            if (max <= 0.0f) return;
            Scale(source, count, 1.0f / max);
        }

        /// <summary>Умножает все отсчёты буфера на заданный коэффициент.</summary>
        public static void Scale(Complex* source, int count, float gain)
        {
            for (var i = 0; i < count; i++)
            {
                source[i].Real *= gain;
                source[i].Imag *= gain;
            }
        }

        /// <summary>
        /// Мягко ограничивает модуль каждого отсчёта единицей, сохраняя фазу.
        /// </summary>
        public static void ClampToUnit(Complex* source, int count)
        {
            for (var i = 0; i < count; i++)
            {
                var modulusSquared = source[i].ModulusSquared();
                if (modulusSquared > 1.0f)
                {
                    source[i] = source[i] * (1.0f / MathF.Sqrt(modulusSquared));
                }
            }
        }

        /// <summary>
        /// Добавляет к каждому отсчёту постоянную составляющую (комплексное смещение).
        /// </summary>
        public static void AddOffset(Complex* source, int count, Complex offset)
        {
            for (var i = 0; i < count; i++)
            {
                source[i].Real += offset.Real;
                source[i].Imag += offset.Imag;
            }
        }

        #endregion
    }
}
