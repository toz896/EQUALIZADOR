# VoiceEQ Mic  (VST 2.x, 64 bits, para Equalizer APO)

EQ de 5 bandas + HPF con 4 slots de voz, noise gate, compresor y de-esser de una perilla cada uno,
limitador de seguridad. Latencia 0 muestras. Pensado para microfonos baratos / de celular.

## Compilar (elige UNA opcion)
1. GitHub Actions (sin instalar nada): sube esta carpeta a un repositorio -> pestana Actions -> descarga `VoiceEQMic-VST2-x64`.
2. Visual Studio Build Tools: abre "x64 Native Tools Command Prompt" y ejecuta `build_msvc.bat`.
3. MinGW-w64 (64 bits): `build_mingw.bat`.

Resultado: `build\VoiceEQMic.dll`  ->  copiala a `C:\Program Files\EqualizerAPO\VSTPlugins\`

## Archivos
- `src/dsp.h`        motor de audio (EQ, gate, compresor, de-esser, limitador)
- `src/params.h`     parametros, rangos y los 4 presets (aqui cambias las voces)
- `src/state.h`      estado compartido sin bloqueos
- `src/plugin.cpp`   plugin VST2 (entrada, dispatcher, chunks, proceso)
- `src/gui_win32.cpp` interfaz (GDI puro)
- `src/vst2_abi.h`   definiciones minimas de la ABI VST2
- `tests/`           pruebas (`tests/run_tests.sh` en Linux/WSL)
