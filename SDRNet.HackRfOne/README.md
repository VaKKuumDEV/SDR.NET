# SDR.NET
Данный набор библиотек предназначен для программной обработки цифровых радиосигналов по технологии SDR. Данная часть для работы с HackRF One. Код взят из репозитория https://github.com/cgommel/sdrsharp и адаптирован под новые версии .NET. В дальнейшем планируется улучшение синтаксиса и оптимизация некоторых функций.

# Использование
Библиотека сделана кросс-платформенной. Для работы некобходимо или установить `hackrf`, или переместить файлы `DLL` в выходной каталог. Библиотека тестировалась и проверялась на Windows и Linux Debian.

# Приём сигнала (RX)
```csharp
using var io = new HackRFIO();
io.Open();               // открыть/выбрать устройство
io.Samplerate = 2_000_000;
io.Frequency = 100_000_000;
io.SamplesAvailable += (sender, buffer, length) =>
{
    // buffer — Complex* на length комплексных отсчётов
};
io.Start();
```

# Передача сигнала (TX)
HackRF One — полудуплексное устройство: одновременно принимать и передавать нельзя.
Перед запуском передачи остановите приём (`Stop()`).

```csharp
using SDRNet.HackRfOne;
using SDRNet.Radio;

using var io = new HackRFIO();
io.Open();                  // выбрать устройство
io.Samplerate = 2_000_000;
io.Frequency = 100_000_000;
io.TxVGAGain = 20;          // 0–47 дБ
// io.EnableAmp = true;     // только с антенной/аттенюатором!

var phase = 0.0;
io.TxSamplesNeeded += (sender, e) =>
{
    // Генерируем тон и заполняем предоставленный буфер.
    phase = SignalGenerator.GenerateTone(
        e.Buffer, e.Length, io.Samplerate, 1_000.0, 0.5f, phase);
};

io.StartTransmit();
// ... передача идёт ...
io.StopTransmit();
```

> Внимание: при включённом усилителе и высокой мощности устройство греется.
> Всегда используйте антенну или эквивалент нагрузки, иначе выходной каскад может выйти из строя.

# Преобразование отсчётов
Класс `SampleConverter` преобразует данные между форматами HackRF (int8, чередование Q/I),
звуковых карт (int16) и комплексными отсчётами `Complex`.

```csharp
// int8 <-> Complex
Complex[] samples = SampleConverter.Int8ToComplex(int8Bytes);
byte[] raw = SampleConverter.ComplexToInt8(samples);

// int16 (звуковая карта / WAV) <-> Complex
SampleConverter.Int16ToComplex(shortPtr, count, complexPtr);
SampleConverter.ComplexToInt16(complexPtr, count, shortPtr);

// Нормализация и ограничение амплитуды
SampleConverter.Normalize(complexPtr, count);
SampleConverter.ClampToUnit(complexPtr, count);
```

# Генерация сигналов
Класс `SignalGenerator` формирует тестовые сигналы в буферы комплексных отсчётов.

```csharp
var phase = 0.0;
phase = SignalGenerator.GenerateTone(buffer, length, sampleRate, 1000.0, 0.5f, phase); // тон
SignalGenerator.GenerateNoise(buffer, length, 0.2f);                                   // шум
phase = SignalGenerator.ShiftFrequency(buffer, length, sampleRate, 250.0, phase);      // сдвиг спектра
SignalGenerator.ApplyRamp(buffer, length, 256);                                        // плавный старт/стоп
```

# Благодарности
Библиотеки грязно слизал отсюда: https://github.com/Infarh/MathCore.HackRF/tree/dev/MathCore.HackRF.  
Конечно же, спасибо мне за мой оптимизм популяризации цифровой обработки сигналов. Библиотеки сделаны для моего научного проекта на чистейшем энтузиазме. Еще моей жене благодарности за терпение.
