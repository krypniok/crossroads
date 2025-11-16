#define _POSIX_C_SOURCE 200809L

#include <AL/al.h>
#include <AL/alc.h>

#if defined(__has_include)
#  if __has_include(<soundtouch/SoundTouchC.h>)
#    include <soundtouch/SoundTouchC.h>
#  elif __has_include(<SoundTouchC.h>)
#    include <SoundTouchC.h>
#  else
#    define SOUND_TOUCH_C_FALLBACK 1
#  endif
#else
#  include <soundtouch/SoundTouchC.h>
#endif

#ifdef SOUND_TOUCH_C_FALLBACK
typedef void *soundtouch_handle_t;
#endif

#include <sndfile.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stddef.h>
#include <unistd.h>
#include <dlfcn.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#define SHORT_PATH_LEN 192

static void copy_string_safe(char *dest, size_t destSize, const char *src) {
    if (destSize == 0) {
        return;
    }
    if (!src) {
        src = "";
    }
    size_t len = strlen(src);
    if (len >= destSize) {
        len = destSize - 1;
    }
    memcpy(dest, src, len);
    dest[len] = '\0';
}

static void copy_short_path(char *dest, size_t destSize, const char *path) {
    copy_string_safe(dest, destSize, path);
}

#define CHECK_AL(msg)                                                           \
    do {                                                                        \
        ALenum err = alGetError();                                              \
        if (err != AL_NO_ERROR) {                                               \
            fprintf(stderr, "OpenAL error (%d) at %s:%d: %s\n", (int)err,       \
                    __FILE__, __LINE__, msg);                                   \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

typedef struct {
    char **items;
    size_t count;
    size_t capacity;
} Playlist;

typedef struct {
    int16_t *samples;
    size_t sampleCount; /* interleaved samples */
    int channels;
    int sampleRate;
    double duration;
    float bpm;
    char path[PATH_MAX];
} PreparedAudio;

typedef struct {
    ALuint buffer;
    ALuint source;
    double duration;
    int valid;
    char path[PATH_MAX];
} TrackSlot;

typedef struct {
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int stop;
    int requestPending;
    char requestPath[PATH_MAX];
    float requestTargetBpm;
    int resultReady;
    int hasError;
    char error[256];
    PreparedAudio prepared;
} LoaderState;

typedef struct {
    int running;
    double startTime;
    float duration;
} CrossfadeState;

typedef soundtouch_handle_t (*soundtouch_createInstance_fn)(void);
typedef void (*soundtouch_destroyInstance_fn)(soundtouch_handle_t);
typedef void (*soundtouch_setSampleRate_fn)(soundtouch_handle_t, unsigned int);
typedef void (*soundtouch_setChannels_fn)(soundtouch_handle_t, unsigned int);
typedef void (*soundtouch_setPitch_fn)(soundtouch_handle_t, float);
typedef void (*soundtouch_setTempo_fn)(soundtouch_handle_t, float);
typedef void (*soundtouch_putSamples_fn)(soundtouch_handle_t, const float *, unsigned int);
typedef unsigned int (*soundtouch_receiveSamples_fn)(soundtouch_handle_t, float *, unsigned int);
typedef void (*soundtouch_flush_fn)(soundtouch_handle_t);

typedef struct {
    void *handle;
    int loaded;
    int attempted;
    char error[256];
    soundtouch_createInstance_fn createInstance;
    soundtouch_destroyInstance_fn destroyInstance;
    soundtouch_setSampleRate_fn setSampleRate;
    soundtouch_setChannels_fn setChannels;
    soundtouch_setPitch_fn setPitch;
    soundtouch_setTempo_fn setTempo;
    soundtouch_putSamples_fn putSamples;
    soundtouch_receiveSamples_fn receiveSamples;
    soundtouch_flush_fn flush;
} SoundTouchAPI;

static SoundTouchAPI g_soundtouch_api;

static volatile sig_atomic_t g_shouldQuit = 0;

static void handle_sigint(int sig) {
    (void)sig;
    g_shouldQuit = 1;
}

static double now_sec(void) {
    struct timespec ts;
#ifdef CLOCK_MONOTONIC
    clock_gettime(CLOCK_MONOTONIC, &ts);
#else
    clock_gettime(CLOCK_REALTIME, &ts);
#endif
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

#define LOAD_ST_SYMBOL(name)                                                    \
    do {                                                                        \
        dlerror();                                                              \
        *(void **)(&g_soundtouch_api.name) = dlsym(g_soundtouch_api.handle,     \
                                                   "soundtouch_" #name);        \
        const char *symErr = dlerror();                                         \
        if (symErr || !g_soundtouch_api.name) {                                 \
            snprintf(g_soundtouch_api.error, sizeof(g_soundtouch_api.error),    \
                     "SoundTouch Symbol %s fehlt (%s)",                         \
                     "soundtouch_" #name, symErr ? symErr : "unbekannt");       \
            if (errBuf && errSize > 0) {                                        \
                snprintf(errBuf, errSize, "%s", g_soundtouch_api.error);        \
            }                                                                   \
            dlclose(g_soundtouch_api.handle);                                   \
            g_soundtouch_api.handle = NULL;                                     \
            return -1;                                                          \
        }                                                                       \
    } while (0)

static int ensure_soundtouch_api(char *errBuf, size_t errSize) {
    if (g_soundtouch_api.loaded) {
        return 0;
    }
    if (g_soundtouch_api.attempted) {
        if (errBuf && errSize > 0) {
            snprintf(errBuf, errSize, "%s", g_soundtouch_api.error[0] ? g_soundtouch_api.error
                                                                     : "SoundTouch API nicht verfügbar");
        }
        return -1;
    }
    g_soundtouch_api.attempted = 1;
    const char *candidates[] = {
        "libSoundTouchDll.so.1",
        "libSoundTouchDll.so",
        "libsoundtouchdll.so.1",
        "libsoundtouchdll.so",
        "libSoundTouch.so.1",
        "libSoundTouch.so",
        "libsoundtouch.so.1",
        "libsoundtouch.so",
        NULL,
    };
    for (size_t i = 0; candidates[i]; ++i) {
        g_soundtouch_api.handle = dlopen(candidates[i], RTLD_LAZY | RTLD_LOCAL);
        if (g_soundtouch_api.handle) {
            break;
        }
    }
    if (!g_soundtouch_api.handle) {
        const char *dlErr = dlerror();
        snprintf(g_soundtouch_api.error, sizeof(g_soundtouch_api.error),
                 "SoundTouch Bibliothek nicht gefunden (%s)", dlErr ? dlErr : "unbekannt");
        if (errBuf && errSize > 0) {
            snprintf(errBuf, errSize, "%s", g_soundtouch_api.error);
        }
        return -1;
    }
    LOAD_ST_SYMBOL(createInstance);
    LOAD_ST_SYMBOL(destroyInstance);
    LOAD_ST_SYMBOL(setSampleRate);
    LOAD_ST_SYMBOL(setChannels);
    LOAD_ST_SYMBOL(setPitch);
    LOAD_ST_SYMBOL(setTempo);
    LOAD_ST_SYMBOL(putSamples);
    LOAD_ST_SYMBOL(receiveSamples);
    LOAD_ST_SYMBOL(flush);
    g_soundtouch_api.loaded = 1;
    return 0;
}

static char *trim_line(char *line) {
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
    size_t start = 0;
    while (line[start] == ' ' || line[start] == '\t') {
        start++;
    }
    if (start > 0) {
        memmove(line, line + start, len - start + 1);
    }
    return line;
}

static int playlist_add(Playlist *playlist, const char *path) {
    if (playlist->count == playlist->capacity) {
        size_t newCap = playlist->capacity == 0 ? 16 : playlist->capacity * 2;
        char **items = realloc(playlist->items, newCap * sizeof(char *));
        if (!items) {
            return -1;
        }
        playlist->items = items;
        playlist->capacity = newCap;
    }
    playlist->items[playlist->count] = strdup(path);
    if (!playlist->items[playlist->count]) {
        return -1;
    }
    playlist->count++;
    return 0;
}

static void playlist_free(Playlist *playlist) {
    for (size_t i = 0; i < playlist->count; ++i) {
        free(playlist->items[i]);
    }
    free(playlist->items);
    playlist->items = NULL;
    playlist->count = playlist->capacity = 0;
}

static int playlist_load(const char *playlistPath, Playlist *playlist, char *errBuf, size_t errSize) {
    FILE *fp = fopen(playlistPath, "r");
    if (!fp) {
        snprintf(errBuf, errSize, "Kann Playlist nicht öffnen: %s", strerror(errno));
        return -1;
    }
    char line[4096];
    size_t lineNum = 0;
    while (fgets(line, sizeof(line), fp)) {
        lineNum++;
        trim_line(line);
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }
        if (playlist_add(playlist, line) != 0) {
            fclose(fp);
            snprintf(errBuf, errSize, "Speicher erschöpft bei Zeile %zu", lineNum);
            return -1;
        }
    }
    fclose(fp);
    if (playlist->count == 0) {
        char shortPath[SHORT_PATH_LEN];
        copy_short_path(shortPath, sizeof(shortPath), playlistPath);
        snprintf(errBuf, errSize, "Playlist %s ist leer", shortPath);
        return -1;
    }
    return 0;
}

static void shuffle_playlist(Playlist *playlist) {
    if (playlist->count < 2) {
        return;
    }
    for (size_t i = playlist->count - 1; i > 0; --i) {
        size_t j = (size_t)(rand() % (i + 1));
        char *tmp = playlist->items[i];
        playlist->items[i] = playlist->items[j];
        playlist->items[j] = tmp;
    }
}

static int read_audio_file(const char *path, float **outSamples, size_t *outFrames, int *outChannels, int *outSampleRate, char *errBuf, size_t errSize) {
    SF_INFO info;
    memset(&info, 0, sizeof(info));
    SNDFILE *sf = sf_open(path, SFM_READ, &info);
    char shortPath[SHORT_PATH_LEN];
    copy_short_path(shortPath, sizeof(shortPath), path);
    if (!sf) {
        snprintf(errBuf, errSize, "sndfile Fehler für %s: %s", shortPath, sf_strerror(NULL));
        return -1;
    }
    if (info.channels < 1 || info.channels > 2) {
        snprintf(errBuf, errSize, "Unterstütze nur 1-2 Kanäle (%s hat %d)", shortPath, info.channels);
        sf_close(sf);
        return -1;
    }
    size_t frames = (size_t)info.frames;
    size_t sampleCount = frames * (size_t)info.channels;
    float *buffer = (float *)malloc(sampleCount * sizeof(float));
    if (!buffer) {
        snprintf(errBuf, errSize, "Speicher erschöpft beim Laden von %s", shortPath);
        sf_close(sf);
        return -1;
    }
    sf_count_t readFrames = sf_readf_float(sf, buffer, (sf_count_t)frames);
    sf_close(sf);
    if (readFrames <= 0) {
        free(buffer);
        snprintf(errBuf, errSize, "Keine Samples in %s gelesen", shortPath);
        return -1;
    }
    *outSamples = buffer;
    *outFrames = (size_t)readFrames;
    *outChannels = info.channels;
    *outSampleRate = info.samplerate;
    return 0;
}

static float detect_bpm(const float *samples, size_t frames, int channels, int sampleRate) {
    if (frames == 0) {
        return 120.0f;
    }
    size_t blockSize = (size_t)(sampleRate / 20); /* 50ms Fenster */
    if (blockSize < 1) {
        blockSize = 1;
    }
    size_t blockCount = frames / blockSize;
    if (blockCount < 4) {
        return 120.0f;
    }
    float *energy = (float *)calloc(blockCount, sizeof(float));
    if (!energy) {
        return 120.0f;
    }
    for (size_t i = 0; i < blockCount; ++i) {
        size_t start = i * blockSize;
        double acc = 0.0;
        for (size_t j = 0; j < blockSize && (start + j) < frames; ++j) {
            size_t idx = (start + j) * (size_t)channels;
            double sampleSum = 0.0;
            for (int c = 0; c < channels; ++c) {
                sampleSum += fabsf(samples[idx + (size_t)c]);
            }
            acc += sampleSum / (double)channels;
        }
        energy[i] = (float)(acc / (double)blockSize);
    }
    double total = 0.0;
    for (size_t i = 0; i < blockCount; ++i) {
        total += energy[i];
    }
    double avg = total / (double)blockCount;
    double threshold = avg * 1.3;
    size_t prevPeak = SIZE_MAX;
    double bpmSum = 0.0;
    size_t bpmCount = 0;
    for (size_t i = 1; i + 1 < blockCount; ++i) {
        if (energy[i] > threshold && energy[i] > energy[i - 1] && energy[i] >= energy[i + 1]) {
            if (prevPeak != SIZE_MAX) {
                size_t diffBlocks = i > prevPeak ? i - prevPeak : prevPeak - i;
                if (diffBlocks > 0) {
                    double intervalSec = (double)(diffBlocks * blockSize) / (double)sampleRate;
                    if (intervalSec > 0.25 && intervalSec < 2.0) {
                        double bpm = 60.0 / intervalSec;
                        if (bpm >= 60.0 && bpm <= 200.0) {
                            bpmSum += bpm;
                            bpmCount++;
                        }
                    }
                }
            }
            prevPeak = i;
        }
    }
    free(energy);
    if (bpmCount == 0) {
        return 120.0f;
    }
    return (float)(bpmSum / (double)bpmCount);
}

static void free_prepared_audio(PreparedAudio *prepared) {
    free(prepared->samples);
    prepared->samples = NULL;
    prepared->sampleCount = 0;
}

static int prepare_audio(const char *path, float targetBpm, PreparedAudio *out, char *errBuf, size_t errSize) {
    memset(out, 0, sizeof(*out));
    float *raw = NULL;
    size_t frames = 0;
    int channels = 0;
    int sampleRate = 0;
    char shortPath[SHORT_PATH_LEN];
    copy_short_path(shortPath, sizeof(shortPath), path);
    if (read_audio_file(path, &raw, &frames, &channels, &sampleRate, errBuf, errSize) != 0) {
        return -1;
    }
    if (ensure_soundtouch_api(errBuf, errSize) != 0) {
        free(raw);
        return -1;
    }
    float detectedBpm = detect_bpm(raw, frames, channels, sampleRate);
    if (targetBpm <= 0.0f) {
        targetBpm = detectedBpm;
    }
    /* Beatmatch via tempo ratio while keeping pitch locked */
    float tempoRatio = detectedBpm > 0.0f ? targetBpm / detectedBpm : 1.0f;
    if (tempoRatio < 0.5f) {
        tempoRatio = 0.5f;
    } else if (tempoRatio > 2.0f) {
        tempoRatio = 2.0f;
    }
    soundtouch_handle_t st = g_soundtouch_api.createInstance();
    if (!st) {
        snprintf(errBuf, errSize, "SoundTouch konnte nicht initialisiert werden");
        free(raw);
        return -1;
    }
    g_soundtouch_api.setSampleRate(st, (unsigned int)sampleRate);
    g_soundtouch_api.setChannels(st, (unsigned int)channels);
    g_soundtouch_api.setPitch(st, 1.0f);
    g_soundtouch_api.setTempo(st, tempoRatio);
    g_soundtouch_api.putSamples(st, raw, (unsigned int)frames);
    size_t capacityFrames = (size_t)((double)frames / (double)tempoRatio) + (size_t)sampleRate;
    if (capacityFrames < frames) {
        capacityFrames = frames;
    }
    float *processed = (float *)malloc(capacityFrames * (size_t)channels * sizeof(float));
    if (!processed) {
        snprintf(errBuf, errSize, "Speicher erschöpft bei SoundTouch");
        g_soundtouch_api.destroyInstance(st);
        free(raw);
        return -1;
    }
    size_t processedFrames = 0;
    while (1) {
        unsigned int received = g_soundtouch_api.receiveSamples(st, processed + processedFrames * (size_t)channels,
                                                         (unsigned int)(capacityFrames - processedFrames));
        if (received == 0) {
            break;
        }
        processedFrames += received;
        if (processedFrames >= capacityFrames) {
            capacityFrames *= 2;
            float *tmp = (float *)realloc(processed, capacityFrames * (size_t)channels * sizeof(float));
            if (!tmp) {
                snprintf(errBuf, errSize, "Speicher erschöpft während Stretch");
                free(processed);
                g_soundtouch_api.destroyInstance(st);
                free(raw);
                return -1;
            }
            processed = tmp;
        }
    }
    g_soundtouch_api.flush(st);
    while (1) {
        unsigned int received = g_soundtouch_api.receiveSamples(st, processed + processedFrames * (size_t)channels,
                                                         (unsigned int)(capacityFrames - processedFrames));
        if (received == 0) {
            break;
        }
        processedFrames += received;
        if (processedFrames >= capacityFrames) {
            capacityFrames *= 2;
            float *tmp = (float *)realloc(processed, capacityFrames * (size_t)channels * sizeof(float));
            if (!tmp) {
                snprintf(errBuf, errSize, "Speicher erschöpft beim Flush");
                free(processed);
                g_soundtouch_api.destroyInstance(st);
                free(raw);
                return -1;
            }
            processed = tmp;
        }
    }
    g_soundtouch_api.destroyInstance(st);
    free(raw);
    if (processedFrames == 0) {
        free(processed);
        snprintf(errBuf, errSize, "Keine Samples nach Time-Stretch für %s", shortPath);
        return -1;
    }
    size_t processedSamples = processedFrames * (size_t)channels;
    int16_t *pcm16 = (int16_t *)malloc(processedSamples * sizeof(int16_t));
    if (!pcm16) {
        free(processed);
        snprintf(errBuf, errSize, "Speicher erschöpft bei PCM16 Konvertierung");
        return -1;
    }
    for (size_t i = 0; i < processedSamples; ++i) {
        float value = processed[i];
        if (value > 1.0f) {
            value = 1.0f;
        } else if (value < -1.0f) {
            value = -1.0f;
        }
        pcm16[i] = (int16_t)lrintf(value * 32767.0f);
    }
    free(processed);
    memset(out, 0, sizeof(*out));
    out->samples = pcm16;
    out->sampleCount = processedSamples;
    out->channels = channels;
    out->sampleRate = sampleRate;
    out->duration = (double)processedFrames / (double)sampleRate;
    out->bpm = detectedBpm;
    copy_string_safe(out->path, sizeof(out->path), path);
    return 0;
}

static ALenum deduce_format(int channels) {
    switch (channels) {
    case 1:
        return AL_FORMAT_MONO16;
    case 2:
        return AL_FORMAT_STEREO16;
    default:
        return 0;
    }
}

static void destroy_slot(TrackSlot *slot) {
    if (!slot->valid) {
        return;
    }
    alSourceStop(slot->source);
    CHECK_AL("alSourceStop");
    alSourcei(slot->source, AL_BUFFER, 0);
    CHECK_AL("alSourcei detach");
    alDeleteSources(1, &slot->source);
    CHECK_AL("alDeleteSources");
    alDeleteBuffers(1, &slot->buffer);
    CHECK_AL("alDeleteBuffers");
    slot->source = 0;
    slot->buffer = 0;
    slot->valid = 0;
    slot->duration = 0.0;
    slot->path[0] = '\0';
}

static void configure_source(ALuint source) {
    alSourcef(source, AL_GAIN, 1.0f);
    alSourcef(source, AL_PITCH, 1.0f);
    alSourcei(source, AL_LOOPING, AL_FALSE);
    CHECK_AL("configure_source");
}

static int slot_from_audio(TrackSlot *slot, const PreparedAudio *audio, char *errBuf, size_t errSize) {
    destroy_slot(slot);
    ALenum format = deduce_format(audio->channels);
    if (format == 0) {
        snprintf(errBuf, errSize, "Unbekanntes Kanal-Format");
        return -1;
    }
    alGenBuffers(1, &slot->buffer);
    CHECK_AL("alGenBuffers");
    alBufferData(slot->buffer, format, audio->samples, (ALsizei)(audio->sampleCount * sizeof(int16_t)), audio->sampleRate);
    CHECK_AL("alBufferData");
    alGenSources(1, &slot->source);
    CHECK_AL("alGenSources");
    configure_source(slot->source);
    alSourcei(slot->source, AL_BUFFER, slot->buffer);
    CHECK_AL("alSourcei");
    slot->valid = 1;
    slot->duration = audio->duration;
    copy_string_safe(slot->path, sizeof(slot->path), audio->path);
    return 0;
}

static void *loader_thread_main(void *userdata);

static int loader_init(LoaderState *loader) {
    memset(loader, 0, sizeof(*loader));
    pthread_mutex_init(&loader->mutex, NULL);
    pthread_cond_init(&loader->cond, NULL);
    if (pthread_create(&loader->thread, NULL, loader_thread_main, loader) != 0) {
        return -1;
    }
    return 0;
}

static void loader_shutdown(LoaderState *loader) {
    pthread_mutex_lock(&loader->mutex);
    loader->stop = 1;
    pthread_cond_broadcast(&loader->cond);
    pthread_mutex_unlock(&loader->mutex);
    pthread_join(loader->thread, NULL);
    free_prepared_audio(&loader->prepared);
    pthread_mutex_destroy(&loader->mutex);
    pthread_cond_destroy(&loader->cond);
}

static int loader_can_accept(LoaderState *loader) {
    return !loader->requestPending && !loader->resultReady && !loader->hasError;
}

static int loader_request(LoaderState *loader, const char *path, float targetBpm) {
    pthread_mutex_lock(&loader->mutex);
    if (!loader_can_accept(loader)) {
        pthread_mutex_unlock(&loader->mutex);
        return -1;
    }
    copy_string_safe(loader->requestPath, sizeof(loader->requestPath), path);
    loader->requestTargetBpm = targetBpm;
    loader->requestPending = 1;
    pthread_cond_broadcast(&loader->cond);
    pthread_mutex_unlock(&loader->mutex);
    return 0;
}

static int loader_wait_result(LoaderState *loader, PreparedAudio *out, char *errBuf, size_t errSize) {
    pthread_mutex_lock(&loader->mutex);
    while (!loader->resultReady && !loader->hasError && !loader->stop) {
        pthread_cond_wait(&loader->cond, &loader->mutex);
    }
    int rc = 0;
    if (loader->resultReady) {
        *out = loader->prepared;
        memset(&loader->prepared, 0, sizeof(loader->prepared));
        loader->resultReady = 0;
        rc = 0;
    } else if (loader->hasError) {
        snprintf(errBuf, errSize, "%s", loader->error);
        loader->hasError = 0;
        rc = -1;
    } else {
        snprintf(errBuf, errSize, "Ladevorgang abgebrochen");
        rc = -1;
    }
    pthread_mutex_unlock(&loader->mutex);
    return rc;
}

static int loader_try_consume(LoaderState *loader, PreparedAudio *out, char *errBuf, size_t errSize) {
    int rc = 0;
    pthread_mutex_lock(&loader->mutex);
    if (loader->resultReady) {
        *out = loader->prepared;
        memset(&loader->prepared, 0, sizeof(loader->prepared));
        loader->resultReady = 0;
        rc = 1;
    } else if (loader->hasError) {
        snprintf(errBuf, errSize, "%s", loader->error);
        loader->hasError = 0;
        rc = -1;
    } else {
        rc = 0;
    }
    pthread_mutex_unlock(&loader->mutex);
    return rc;
}

static void loader_report_error(LoaderState *loader, const char *msg) {
    pthread_mutex_lock(&loader->mutex);
    snprintf(loader->error, sizeof(loader->error), "%s", msg);
    loader->hasError = 1;
    loader->resultReady = 0;
    pthread_cond_broadcast(&loader->cond);
    pthread_mutex_unlock(&loader->mutex);
}

static void loader_publish_result(LoaderState *loader, PreparedAudio *prepared) {
    pthread_mutex_lock(&loader->mutex);
    loader->prepared = *prepared;
    loader->resultReady = 1;
    loader->hasError = 0;
    pthread_cond_broadcast(&loader->cond);
    pthread_mutex_unlock(&loader->mutex);
}

static void *loader_thread_main(void *userdata) {
    LoaderState *loader = (LoaderState *)userdata;
    pthread_mutex_lock(&loader->mutex);
    while (!loader->stop) {
        while (!loader->requestPending && !loader->stop) {
            pthread_cond_wait(&loader->cond, &loader->mutex);
        }
        if (loader->stop) {
            break;
        }
        char path[PATH_MAX];
        copy_string_safe(path, sizeof(path), loader->requestPath);
        float target = loader->requestTargetBpm;
        loader->requestPending = 0;
        pthread_mutex_unlock(&loader->mutex);

        PreparedAudio prepared;
        char err[256];
        if (prepare_audio(path, target, &prepared, err, sizeof(err)) == 0) {
            loader_publish_result(loader, &prepared);
        } else {
            loader_report_error(loader, err);
        }

        pthread_mutex_lock(&loader->mutex);
    }
    pthread_mutex_unlock(&loader->mutex);
    return NULL;
}

static double source_offset_seconds(const TrackSlot *slot) {
    if (!slot->valid) {
        return 0.0;
    }
    float value = 0.0f;
    alGetSourcef(slot->source, AL_SEC_OFFSET, &value);
    CHECK_AL("alGetSourcef");
    return value;
}

static void print_usage(const char *exe) {
    fprintf(stderr, "Verwendung: %s <playlist.m3u> [crossfade_sec] [target_bpm]\n", exe);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    const char *playlistPath = argv[1];
    float crossfadeSeconds = argc >= 3 ? strtof(argv[2], NULL) : 8.0f;
    if (crossfadeSeconds < 1.0f) {
        crossfadeSeconds = 1.0f;
    }
    float targetBpm = argc >= 4 ? strtof(argv[3], NULL) : -1.0f;
    signal(SIGINT, handle_sigint);
    srand((unsigned int)time(NULL));

    Playlist playlist = {0};
    char errBuf[256];
    if (playlist_load(playlistPath, &playlist, errBuf, sizeof(errBuf)) != 0) {
        fprintf(stderr, "%s\n", errBuf);
        return EXIT_FAILURE;
    }
    shuffle_playlist(&playlist);

    ALCdevice *device = alcOpenDevice(NULL);
    if (!device) {
        fprintf(stderr, "Konnte kein OpenAL-Device öffnen\n");
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }
    ALCcontext *context = alcCreateContext(device, NULL);
    if (!context) {
        fprintf(stderr, "Konnte kein OpenAL-Kontext erstellen\n");
        alcCloseDevice(device);
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }
    alcMakeContextCurrent(context);

    LoaderState loader;
    if (loader_init(&loader) != 0) {
        fprintf(stderr, "Konnte Lade-Thread nicht starten\n");
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context);
        alcCloseDevice(device);
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }

    TrackSlot slots[2];
    memset(slots, 0, sizeof(slots));
    int currentSlot = 0;
    int standbySlot = 1;
    int standbyReady = 0;
    size_t standbyTrackIndex = (size_t)-1;
    size_t pendingIndex = (size_t)-1;
    size_t playlistIndex = 0;
    size_t nextIndex = playlist.count > 1 ? 1 : 0;
    float globalTargetBpm = targetBpm;
    CrossfadeState crossfade = {0};
    int loaderBusy = 0;

    if (loader_request(&loader, playlist.items[playlistIndex], targetBpm) != 0) {
        fprintf(stderr, "Erste Datei konnte nicht geladen werden\n");
        loader_shutdown(&loader);
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context);
        alcCloseDevice(device);
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }
    loaderBusy = 1;
    pendingIndex = playlistIndex;

    PreparedAudio prepared;
    if (loader_wait_result(&loader, &prepared, errBuf, sizeof(errBuf)) != 0) {
        fprintf(stderr, "Fehler beim Laden des ersten Titels: %s\n", errBuf);
        loader_shutdown(&loader);
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context);
        alcCloseDevice(device);
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }
    loaderBusy = 0;
    pendingIndex = (size_t)-1;
    if (globalTargetBpm <= 0.0f) {
        globalTargetBpm = prepared.bpm;
    }
    if (slot_from_audio(&slots[currentSlot], &prepared, errBuf, sizeof(errBuf)) != 0) {
        fprintf(stderr, "OpenAL Buffer Fehler: %s\n", errBuf);
        free_prepared_audio(&prepared);
        loader_shutdown(&loader);
        alcMakeContextCurrent(NULL);
        alcDestroyContext(context);
        alcCloseDevice(device);
        playlist_free(&playlist);
        return EXIT_FAILURE;
    }
    printf("Starte: %s (BPM %.1f -> Ziel %.1f)\n", slots[currentSlot].path, prepared.bpm, globalTargetBpm);
    free_prepared_audio(&prepared);
    alSourcePlay(slots[currentSlot].source);
    CHECK_AL("alSourcePlay");

    int running = 1;
    while (running && !g_shouldQuit) {
        if (!standbyReady && !loaderBusy && playlist.count > 0) {
            const char *nextPath = playlist.items[nextIndex];
            if (loader_request(&loader, nextPath, globalTargetBpm) == 0) {
                loaderBusy = 1;
                pendingIndex = nextIndex;
                printf("Lade vor: %s\n", nextPath);
            }
        }

        if (loaderBusy) {
            PreparedAudio ready;
            int state = loader_try_consume(&loader, &ready, errBuf, sizeof(errBuf));
            if (state == 1) {
                if (slot_from_audio(&slots[standbySlot], &ready, errBuf, sizeof(errBuf)) == 0) {
                    standbyReady = 1;
                    standbyTrackIndex = pendingIndex;
                    printf("Bereit für Crossfade: %s\n", slots[standbySlot].path);
                    nextIndex = (standbyTrackIndex + 1) % playlist.count;
                } else {
                    fprintf(stderr, "OpenAL Fehler beim Standby: %s\n", errBuf);
                }
                free_prepared_audio(&ready);
                loaderBusy = 0;
                pendingIndex = (size_t)-1;
            } else if (state == -1) {
                fprintf(stderr, "Fehler beim Laden: %s\n", errBuf);
                loaderBusy = 0;
                if (pendingIndex != (size_t)-1) {
                    nextIndex = (pendingIndex + 1) % playlist.count;
                    pendingIndex = (size_t)-1;
                }
            }
        }

        if (standbyReady && slots[currentSlot].valid && !crossfade.running) {
            double elapsed = source_offset_seconds(&slots[currentSlot]);
            double remaining = slots[currentSlot].duration - elapsed;
            if (remaining < crossfadeSeconds + 0.25) {
                float effectiveFade = crossfadeSeconds;
                if (slots[currentSlot].duration < effectiveFade) {
                    effectiveFade = (float)(slots[currentSlot].duration * 0.5);
                }
                if (slots[standbySlot].duration < effectiveFade) {
                    effectiveFade = (float)(slots[standbySlot].duration * 0.8);
                }
                if (effectiveFade < 1.0f) {
                    effectiveFade = 1.0f;
                }
                alSourcef(slots[currentSlot].source, AL_GAIN, 1.0f);
                alSourcef(slots[standbySlot].source, AL_GAIN, 0.0f);
                alSourcePlay(slots[standbySlot].source);
                CHECK_AL("alSourcePlay (next)");
                crossfade.running = 1;
                crossfade.duration = effectiveFade;
                crossfade.startTime = now_sec();
                printf("Crossfade %.2fs: %s -> %s\n", effectiveFade, slots[currentSlot].path, slots[standbySlot].path);
            }
        }

        if (crossfade.running) {
            double elapsed = now_sec() - crossfade.startTime;
            float t = (float)(elapsed / (double)crossfade.duration);
            if (t > 1.0f) {
                t = 1.0f;
            }
            alSourcef(slots[currentSlot].source, AL_GAIN, 1.0f - t);
            alSourcef(slots[standbySlot].source, AL_GAIN, t);
            if (elapsed >= crossfade.duration) {
                alSourcef(slots[currentSlot].source, AL_GAIN, 0.0f);
                alSourceStop(slots[currentSlot].source);
                CHECK_AL("alSourceStop (old)");
                int old = currentSlot;
                currentSlot = standbySlot;
                standbySlot = old;
                destroy_slot(&slots[standbySlot]);
                playlistIndex = standbyTrackIndex;
                standbyTrackIndex = (size_t)-1;
                standbyReady = 0;
                crossfade.running = 0;
            }
        } else {
            ALint state;
            alGetSourcei(slots[currentSlot].source, AL_SOURCE_STATE, &state);
            CHECK_AL("alGetSourcei");
            if (state != AL_PLAYING && standbyReady) {
                alSourcef(slots[standbySlot].source, AL_GAIN, 1.0f);
                alSourcePlay(slots[standbySlot].source);
                CHECK_AL("alSourcePlay immediate");
                currentSlot = standbySlot;
                standbySlot = 1 - standbySlot;
                destroy_slot(&slots[standbySlot]);
                playlistIndex = standbyTrackIndex;
                standbyTrackIndex = (size_t)-1;
                standbyReady = 0;
            } else if (state != AL_PLAYING && !standbyReady && !loaderBusy) {
                printf("Warte auf nächstes Lied...\n");
            }
        }

        sleep_ms(50);
        running = playlist.count > 0;
    }

    destroy_slot(&slots[0]);
    destroy_slot(&slots[1]);
    loader_shutdown(&loader);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(context);
    alcCloseDevice(device);
    playlist_free(&playlist);
    printf("Beendet.\n");
    return EXIT_SUCCESS;
}
