#!/usr/bin/env bash
set -uo pipefail

SRC_DIR="${SRC_DIR:-/media/simon/PortableSSD/simon-media/Mediathek/Musikvideos}"
DEST_DIR="${DEST_DIR:-$(pwd)/mp3_library}"
PLAYLIST_PATH="${PLAYLIST_PATH:-$(pwd)/playlist.m3u}"
ERROR_LOG="$DEST_DIR/conversion_errors.log"
FORCE_REENCODE="${FORCE_REENCODE:-0}"   # 1 = überschreibe vorhandene Dateien (zum Reparieren)

mkdir -p "$DEST_DIR"
> "$ERROR_LOG"

convert_file() {
    local src="$1"
    local base
    base="$(basename "$src")"
    local name="${base%.*}"
    local dest="$DEST_DIR/$name.mp3"
    local display="${src#$SRC_DIR/}"
    if [[ -f "$dest" && "$FORCE_REENCODE" != "1" ]]; then
        printf 'Überspringe vorhandene Datei: %s (FORCE_REENCODE=1 zum Überschreiben)\n' "$dest"
        return 0
    fi
    printf 'Konvertiere/Repariere: %s -> %s\n' "$display" "$dest"
    if ffmpeg -y -hide_banner -nostdin -loglevel error \
        -err_detect ignore_err -i "$src" \
        -vn -ac 2 -ar 44100 -af aresample=async=1 \
        -c:a libmp3lame -b:a 256k \
        -map_metadata 0 -id3v2_version 3 \
        "$dest"; then
        return 0
    else
        printf 'Fehler bei %s – siehe %s\n' "$display" "$ERROR_LOG"
        printf '%s\n' "$src" >> "$ERROR_LOG"
        rm -f "$dest"
        return 0
    fi
}

find "$SRC_DIR" -maxdepth 1 -type f \( \
    -iname '*.webm' -o \
    -iname '*.mkv' -o \
    -iname '*.mp4' -o \
    -iname '*.m4a' -o \
    -iname '*.mp3' \
\) -print0 | while IFS= read -r -d '' file; do
    convert_file "$file"
done

find "$DEST_DIR" -maxdepth 1 -type f -name '*.mp3' -print | sort > "$PLAYLIST_PATH"
printf 'Playlist geschrieben nach %s\n' "$PLAYLIST_PATH"
if [[ -s "$ERROR_LOG" ]]; then
    printf 'Einige Dateien konnten nicht konvertiert werden. Details: %s\n' "$ERROR_LOG"
else
    rm -f "$ERROR_LOG"
fi
