# crossroads

Crossfade-/Beatmatch-Player mit zwei Decks (OpenAL + SoundTouch + GTK).

## Features
- Zwei Decks (A/B) mit Walkman-Transport (Prev/Play/Pause/Stop/Next), separaten Track-Listen und optionalem Autoplay des nächsten Tracks
- Crossfader (A↔B) plus Auto-Crossfade-Button mit einstellbarer Dauer
- Beatmatch (Pitch-Lock) schaltbar; globaler Ziel-BPM-Slider (60–200) + Sync
- Per-Deck Tempo-Slider (0.5–2.0) und Pitch-Slider (0.5–2.0)
- Per-Deck Gain (Volume) und 10-Band-EQ plus Master-10-Band-EQ
- Dynamischer EQ: moduliert den Master-EQ in Sinus/Zickzack/PWM (Amplitude+Speed regelbar)
- Seek-Slider je Deck (läuft mit), „Play Sample“-Knopf mischt einen kurzen Testton dazu
- Master-Gain, Beatmatch via SoundTouch, Streaming ohne Vollvorladen

## Build
```sh
make            # erzeugt ./crossroads
```
Benötigt: `libopenal`, `libsndfile`, `libsoundtouch`, `gtk+-3.0` Dev-Pakete.

## Run
```sh
./crossroads             # nutzt mp3_library als Quelle
./crossroads /pfad/zu/mp3s
```
- Track in Deck A/B wählen, „Play Deck …“ starten; Play/Pause/Stop pro Deck nutzen.
- Crossfader stellen oder Auto-Crossfade nutzen; Beatmatch an, Ziel-BPM setzen, „Sync to BPM“ drücken; je Deck Tempo/Pitch/Gain/EQ feinjustieren.
- Seek-Slider springt sofort; Sample-Button mischt einen Testton ohne die Decks zu stoppen.
- Dynamischer EQ im Master an/aus + Wellenform (Sinus/Zickzack/PWM), Speed und Tiefe einstellen; die Master-EQ-Slider zeigen die Bewegung live.

## Hinweise
- MP3 werden on-the-fly gestreamt; bei Beatmatch rechnet SoundTouch blockweise. Puffer: 2048 Frames ×4; bei Aussetzern erhöhen.
- Auto-Crossfade bewegt den Crossfader A→B; stelle sicher, dass Deck B spielt.
