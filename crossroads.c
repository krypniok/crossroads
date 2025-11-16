#define _POSIX_C_SOURCE 200809L

#include <AL/al.h>
#include <AL/alc.h>
#include <gtk/gtk.h>
#include <sndfile.h>

#include <dlfcn.h>
#include <dirent.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STREAM_BUFFER_FRAMES 1024
#define STREAM_BUFFERS 6

typedef void *soundtouch_handle_t;
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

static SoundTouchAPI g_st;
static volatile sig_atomic_t g_should_quit = 0;

typedef struct {
    char **items;
    size_t count;
} Playlist;

typedef struct {
    SNDFILE *sf;
    SF_INFO info;
    soundtouch_handle_t st;
    ALuint source;
    ALuint buffers[STREAM_BUFFERS];
    int buf_index;
    int eof_sent;
    int finished;
    int beatmatch;
    float detected_bpm;
    float target_bpm;
    float tempo_ratio;
    float tempo_mul;
    float pitch_mul;
    float gain_mul;
    double position;
    double last_pos_update;
    float eq_low;
    float eq_mid;
    float eq_high;
    float eq_lp_state[2];
    float eq_hp_state[2];
    double duration;
    char path[PATH_MAX];
} Deck;

typedef enum {
    CMD_PLAY_A,
    CMD_PLAY_B,
    CMD_STOP_A,
    CMD_STOP_B,
    CMD_SEEK_A,
    CMD_SEEK_B,
    CMD_SET_CROSSFADER,
    CMD_SET_TARGET_BPM,
    CMD_SET_BEATMATCH,
    CMD_SYNC_NOW,
    CMD_AUTO_XFADE,
    CMD_PLAY_PAUSE_A,
    CMD_PLAY_PAUSE_B,
    CMD_TEMPO_A,
    CMD_TEMPO_B,
    CMD_PITCH_A,
    CMD_PITCH_B,
    CMD_EQ_LOW_A,
    CMD_EQ_MID_A,
    CMD_EQ_HIGH_A,
    CMD_EQ_LOW_B,
    CMD_EQ_MID_B,
    CMD_EQ_HIGH_B,
    CMD_PLAY_SAMPLE,
    CMD_GAIN_A,
    CMD_GAIN_B,
    CMD_MASTER_GAIN,
    CMD_QUIT
} CommandType;

typedef struct {
    CommandType type;
    double value;
} Command;

typedef struct {
    GtkWidget *window;
    GtkWidget *list_a;
    GtkWidget *list_b;
    GtkWidget *play_a;
    GtkWidget *play_b;
    GtkWidget *pp_a;
    GtkWidget *pp_b;
    GtkWidget *stop_a;
    GtkWidget *stop_b;
    GtkWidget *seek_a;
    GtkWidget *seek_b;
    GtkWidget *crossfader;
    GtkWidget *bpm_scale;
    GtkWidget *beatmatch_toggle;
    GtkWidget *sync_button;
    GtkWidget *auto_xfade_btn;
    GtkWidget *xfade_time_scale;
    GtkWidget *tempo_a;
    GtkWidget *tempo_b;
    GtkWidget *pitch_a;
    GtkWidget *pitch_b;
    GtkWidget *eq_low_a;
    GtkWidget *eq_mid_a;
    GtkWidget *eq_high_a;
    GtkWidget *eq_low_b;
    GtkWidget *eq_mid_b;
    GtkWidget *eq_high_b;
    GtkWidget *gain_a;
    GtkWidget *gain_b;
    GtkWidget *sample_btn;
    GtkWidget *master_gain;
    GtkWidget *pos_label_a;
    GtkWidget *pos_label_b;
} UI;

static gboolean g_updating_seek = FALSE;

typedef struct {
    Playlist playlist;
    ALCdevice *dev;
    ALCcontext *ctx;
    Deck decks[2];
    double crossfader;
    float target_bpm;
    int beatmatch;
    double auto_xfade_start;
    double auto_xfade_dur;
    int auto_xfade_active;
    float master_gain;
    GAsyncQueue *queue;
} EngineState;

typedef struct {
    EngineState *eng;
    UI *ui;
} UiCtx;

static void safe_copy(char *dst, size_t dstsz, const char *src) {
    if (!dstsz) return;
    if (!src) src = "";
    size_t len = strlen(src);
    if (len >= dstsz) len = dstsz - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int ensure_soundtouch(char *err, size_t errsz) {
    if (g_st.loaded) return 0;
    if (g_st.attempted) {
        if (err && errsz) safe_copy(err, errsz, g_st.error[0] ? g_st.error : "SoundTouch not available");
        return -1;
    }
    g_st.attempted = 1;
    const char *libs[] = {
        "libSoundTouchDll.so.1", "libSoundTouchDll.so",
        "libsoundtouchdll.so.1", "libsoundtouchdll.so",
        "libSoundTouch.so.1", "libSoundTouch.so",
        "libsoundtouch.so.1", "libsoundtouch.so",
        NULL
    };
    for (size_t i = 0; libs[i]; ++i) {
        g_st.handle = dlopen(libs[i], RTLD_LOCAL | RTLD_LAZY);
        if (g_st.handle) break;
    }
    if (!g_st.handle) {
        safe_copy(g_st.error, sizeof(g_st.error), "SoundTouch library not found");
        if (err && errsz) safe_copy(err, errsz, g_st.error);
        return -1;
    }
#define LOAD(name) do {                                                        \
    dlerror();                                                                 \
    *(void**)(&g_st.name) = dlsym(g_st.handle, "soundtouch_" #name);           \
    const char *e = dlerror();                                                 \
    if (e || !g_st.name) {                                                     \
        snprintf(g_st.error, sizeof(g_st.error), "Missing %s", #name);         \
        if (err && errsz) safe_copy(err, errsz, g_st.error);                   \
        dlclose(g_st.handle);                                                  \
        g_st.handle = NULL;                                                    \
        return -1;                                                             \
    }                                                                          \
} while(0)
    LOAD(createInstance); LOAD(destroyInstance); LOAD(setSampleRate); LOAD(setChannels);
    LOAD(setPitch); LOAD(setTempo); LOAD(putSamples); LOAD(receiveSamples); LOAD(flush);
#undef LOAD
    g_st.loaded = 1;
    return 0;
}

static float detect_bpm_quick(SNDFILE *sf, const SF_INFO *info) {
    const int seconds = 5;
    size_t frames = (size_t)info->samplerate * seconds;
    float *buf = malloc(frames * info->channels * sizeof(float));
    if (!buf) return 120.f;
    sf_seek(sf, 0, SEEK_SET);
    sf_count_t got = sf_readf_float(sf, buf, (sf_count_t)frames);
    if (got <= 0) { free(buf); return 120.f; }
    size_t block = (size_t)(info->samplerate / 20);
    if (!block) block = 1;
    size_t blocks = (size_t)got / block;
    if (blocks < 4) { free(buf); return 120.f; }
    float *energy = calloc(blocks, sizeof(float));
    if (!energy) { free(buf); return 120.f; }
    double sum = 0;
    for (size_t i = 0; i < blocks; ++i) {
        double acc = 0;
        for (size_t j = 0; j < block; ++j) {
            size_t idx = (i * block + j) * (size_t)info->channels;
            double v = 0;
            for (int c = 0; c < info->channels; ++c) v += fabsf(buf[idx+c]);
            acc += v / info->channels;
        }
        energy[i] = (float)(acc / block);
        sum += energy[i];
    }
    double avg = sum / blocks;
    double threshold = avg * 1.3;
    size_t prev = SIZE_MAX;
    double bpm_sum = 0; size_t bpm_count = 0;
    for (size_t i = 1; i + 1 < blocks; ++i) {
        if (energy[i] > threshold && energy[i] > energy[i-1] && energy[i] >= energy[i+1]) {
            if (prev != SIZE_MAX) {
                size_t diff = i > prev ? i - prev : prev - i;
                double interval = (double)(diff * block) / info->samplerate;
                if (interval > 0.25 && interval < 2.0) {
                    double bpm = 60.0 / interval;
                    if (bpm >= 60 && bpm <= 200) { bpm_sum += bpm; bpm_count++; }
                }
            }
            prev = i;
        }
    }
    free(energy); free(buf);
    return bpm_count ? (float)(bpm_sum / bpm_count) : 120.f;
}

static void destroy_deck(Deck *d) {
    if (d->source) {
        alSourceStop(d->source);
        ALint q = 0;
        alGetSourcei(d->source, AL_BUFFERS_QUEUED, &q);
        while (q-- > 0) {
            ALuint b; alSourceUnqueueBuffers(d->source, 1, &b);
        }
        alDeleteSources(1, &d->source);
        alDeleteBuffers(STREAM_BUFFERS, d->buffers);
    }
    if (d->sf) sf_close(d->sf);
    if (d->st && g_st.loaded) g_st.destroyInstance(d->st);
    memset(d, 0, sizeof(*d));
    d->tempo_mul = 1.0f;
    d->pitch_mul = 1.0f;
    d->gain_mul = 1.0f;
}

static int init_deck(Deck *d, const char *path, float target_bpm, int beatmatch, char *err, size_t errsz) {
    destroy_deck(d);
    safe_copy(d->path, sizeof(d->path), path);
    d->beatmatch = beatmatch;
    d->target_bpm = target_bpm;
    d->sf = sf_open(path, SFM_READ, &d->info);
    if (!d->sf) { snprintf(err, errsz, "sndfile: %s", sf_strerror(NULL)); return -1; }
    alGenSources(1, &d->source);
    alGenBuffers(STREAM_BUFFERS, d->buffers);
    d->buf_index = 0;
    d->detected_bpm = 0;
    d->tempo_ratio = 1.0f;
    d->tempo_mul = 1.0f;
    d->pitch_mul = 1.0f;
    d->gain_mul = 1.0f;
    d->eq_low = d->eq_mid = d->eq_high = 1.0f;
    d->duration = (double)d->info.frames / (double)d->info.samplerate;
    d->position = 0.0;
    d->last_pos_update = now_sec();
    if (beatmatch) {
        if (ensure_soundtouch(err, errsz) != 0) return -1;
        d->detected_bpm = detect_bpm_quick(d->sf, &d->info);
        float target = target_bpm > 0 ? target_bpm : d->detected_bpm;
        if (target <= 0) target = 120.f;
        d->tempo_ratio = d->detected_bpm > 0 ? target / d->detected_bpm : 1.0f;
        if (d->tempo_ratio < 0.5f) d->tempo_ratio = 0.5f;
        if (d->tempo_ratio > 2.0f) d->tempo_ratio = 2.0f;
        d->st = g_st.createInstance();
        if (!d->st) { snprintf(err, errsz, "SoundTouch init failed"); return -1; }
        g_st.setSampleRate(d->st, (unsigned int)d->info.samplerate);
        g_st.setChannels(d->st, (unsigned int)d->info.channels);
        g_st.setPitch(d->st, 1.0f);
        g_st.setTempo(d->st, d->tempo_ratio);
    }
    sf_seek(d->sf, 0, SEEK_SET);
    alSourcef(d->source, AL_GAIN, 1.0f);
    alSourcei(d->source, AL_LOOPING, AL_FALSE);
    return 0;
}

static void apply_tempo(Deck *d, float target_bpm, int beatmatch) {
    if (!d->st) return;
    if (!beatmatch || d->detected_bpm <= 0) return;
    float ratio = d->detected_bpm > 0 ? target_bpm / d->detected_bpm : 1.0f;
    ratio *= d->tempo_mul;
    if (ratio < 0.5f) ratio = 0.5f;
    if (ratio > 2.0f) ratio = 2.0f;
    g_st.setTempo(d->st, ratio);
    float pitch = d->pitch_mul;
    if (pitch < 0.5f) pitch = 0.5f;
    if (pitch > 2.0f) pitch = 2.0f;
    g_st.setPitch(d->st, pitch);
}

static void queue_more(Deck *d) {
    if (d->finished || !d->source) return;
    ALint queued = 0;
    alGetSourcei(d->source, AL_BUFFERS_QUEUED, &queued);
    while (queued < STREAM_BUFFERS) {
        float processed[STREAM_BUFFER_FRAMES * 2];
        unsigned int got = 0;
        int attempts = 0;
        while (attempts < 8 && got == 0) {
            float fbuf[STREAM_BUFFER_FRAMES * 2];
            sf_count_t r = sf_readf_float(d->sf, fbuf, STREAM_BUFFER_FRAMES);
            if (r <= 0) {
                if (d->st) g_st.flush(d->st);
                d->eof_sent = 1;
            } else if (d->st) {
                g_st.putSamples(d->st, fbuf, (unsigned int)r);
            } else {
                got = (unsigned int)r;
                memcpy(processed, fbuf, (size_t)got * d->info.channels * sizeof(float));
                break;
            }
            if (d->st) {
                got = g_st.receiveSamples(d->st, processed, STREAM_BUFFER_FRAMES);
            }
            attempts++;
        }
        if (got == 0) { if (d->eof_sent) d->finished = 1; break; }
        int16_t pcm[STREAM_BUFFER_FRAMES * 2];
        size_t samples = (size_t)got * (size_t)d->info.channels;
        for (size_t i = 0; i < samples; ++i) {
            float v = processed[i];
            int ch = (int)(i % d->info.channels);
            double dt = 1.0 / (double)d->info.samplerate;
            double rc_low = 1.0 / (2.0 * M_PI * 200.0);
            double rc_high = 1.0 / (2.0 * M_PI * 2000.0);
            double alpha_l = dt / (rc_low + dt);
            double alpha_h = dt / (rc_high + dt);
            double lp = d->eq_lp_state[ch] + alpha_l * (v - d->eq_lp_state[ch]);
            d->eq_lp_state[ch] = (float)lp;
            double hp_in = v - lp;
            double hp = d->eq_hp_state[ch] + alpha_h * (hp_in - d->eq_hp_state[ch]);
            d->eq_hp_state[ch] = (float)hp;
            double mid = v - lp - hp;
            v = (float)(lp * d->eq_low + mid * d->eq_mid + hp * d->eq_high);
            if (v > 1.f) v = 1.f;
            else if (v < -1.f) v = -1.f;
            pcm[i] = (int16_t)lrintf(v * 32767.f);
        }
        ALenum fmt = d->info.channels == 1 ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;
        ALuint buf = d->buffers[d->buf_index++ % STREAM_BUFFERS];
        alBufferData(buf, fmt, pcm, (ALsizei)(samples * sizeof(int16_t)), d->info.samplerate);
        alSourceQueueBuffers(d->source, 1, &buf);
        queued++;
    }
}

static void update_deck(Deck *d) {
    if (!d->source) return;
    double now = now_sec();
    double dt = now - d->last_pos_update;
    ALint processed = 0;
    alGetSourcei(d->source, AL_BUFFERS_PROCESSED, &processed);
    while (processed-- > 0) {
        ALuint b; alSourceUnqueueBuffers(d->source, 1, &b);
    }
    queue_more(d);
    ALint state = AL_STOPPED;
    alGetSourcei(d->source, AL_SOURCE_STATE, &state);
    if (state == AL_PLAYING || state == AL_PAUSED) {
        double tempo_scale = d->tempo_ratio * d->tempo_mul;
        if (tempo_scale < 0.1) tempo_scale = 0.1;
        if (tempo_scale > 4.0) tempo_scale = 4.0;
        d->position += dt * tempo_scale;
        if (d->duration > 0 && d->position > d->duration) d->position = d->duration;
    }
    d->last_pos_update = now;
    if ((d->finished || state == AL_STOPPED) && d->duration > 0 && d->position >= d->duration) {
        d->position = d->duration;
    }
}

static void start_deck(Deck *d) {
    if (!d->source) return;
    d->last_pos_update = now_sec();
    queue_more(d);
    ALint q = 0;
    alGetSourcei(d->source, AL_BUFFERS_QUEUED, &q);
    if (q > 0) alSourcePlay(d->source);
}

static void set_crossfader(EngineState *e, double pos) {
    e->crossfader = pos; /* 0..1 */
    float gain_a = (float)(1.0 - pos) * e->decks[0].gain_mul * e->master_gain;
    float gain_b = (float)(pos) * e->decks[1].gain_mul * e->master_gain;
    alSourcef(e->decks[0].source, AL_GAIN, gain_a);
    alSourcef(e->decks[1].source, AL_GAIN, gain_b);
}

static gpointer sample_thread(gpointer data) {
    (void)data;
    const int sr = 44100;
    const double freq = 440.0;
    const double dur = 1.0;
    size_t frames = (size_t)(sr * dur);
    int16_t *pcm = malloc(frames * 2 * sizeof(int16_t));
    if (!pcm) return NULL;
    for (size_t i = 0; i < frames; ++i) {
        double t = (double)i / sr;
        double s = sin(2.0 * M_PI * freq * t) * 0.3;
        int16_t v = (int16_t)lrintf((float)s * 32767.f);
        pcm[2 * i] = v;
        pcm[2 * i + 1] = v;
    }
    ALuint buf, src;
    alGenBuffers(1, &buf);
    alBufferData(buf, AL_FORMAT_STEREO16, pcm, (ALsizei)(frames * 2 * sizeof(int16_t)), sr);
    alGenSources(1, &src);
    alSourcei(src, AL_BUFFER, buf);
    alSourcef(src, AL_GAIN, 0.6f);
    alSourcePlay(src);
    free(pcm);
    ALint state = AL_PLAYING;
    while (state == AL_PLAYING && !g_should_quit) {
        alGetSourcei(src, AL_SOURCE_STATE, &state);
        g_usleep(10000);
    }
    alSourceStop(src);
    alDeleteSources(1, &src);
    alDeleteBuffers(1, &buf);
    return NULL;
}

static void play_sample_tone(EngineState *e) {
    (void)e;
    g_thread_new("sample", sample_thread, NULL);
}

static void load_and_play_selection(EngineState *e, GtkListBox *list, int deck_id) {
    GtkListBoxRow *row = gtk_list_box_get_selected_row(list);
    if (!row) return;
    int idx = gtk_list_box_row_get_index(row);
    if (idx < 0 || (size_t)idx >= e->playlist.count) return;
    char err[256];
    if (init_deck(&e->decks[deck_id], e->playlist.items[idx], e->target_bpm, e->beatmatch, err, sizeof(err)) == 0) {
        apply_tempo(&e->decks[deck_id], e->target_bpm, e->beatmatch);
        start_deck(&e->decks[deck_id]);
    } else {
        fprintf(stderr, "Deck %d load failed: %s\n", deck_id, err);
    }
}

static gpointer engine_thread(gpointer data) {
    EngineState *e = data;
    double last = now_sec();
    while (!g_should_quit) {
        Command *cmd = g_async_queue_try_pop(e->queue);
        if (cmd) {
            switch (cmd->type) {
            case CMD_PLAY_A: start_deck(&e->decks[0]); break;
            case CMD_PLAY_B: start_deck(&e->decks[1]); break;
            case CMD_PLAY_PAUSE_A: {
                ALint state = AL_STOPPED;
                alGetSourcei(e->decks[0].source, AL_SOURCE_STATE, &state);
                if (state == AL_PLAYING) alSourcePause(e->decks[0].source);
                else start_deck(&e->decks[0]);
                break;
            }
            case CMD_PLAY_PAUSE_B: {
                ALint state = AL_STOPPED;
                alGetSourcei(e->decks[1].source, AL_SOURCE_STATE, &state);
                if (state == AL_PLAYING) alSourcePause(e->decks[1].source);
                else start_deck(&e->decks[1]);
                break;
            }
            case CMD_STOP_A: destroy_deck(&e->decks[0]); break;
            case CMD_STOP_B: destroy_deck(&e->decks[1]); break;
            case CMD_SEEK_A:
                if (e->decks[0].sf) {
                    sf_seek(e->decks[0].sf, (sf_count_t)(cmd->value * e->decks[0].info.samplerate), SEEK_SET);
                    if (e->decks[0].st) g_st.flush(e->decks[0].st);
                    e->decks[0].buf_index = 0;
                    e->decks[0].finished = 0;
                    e->decks[0].position = cmd->value;
                    e->decks[0].last_pos_update = now_sec();
                    start_deck(&e->decks[0]);
                }
                break;
            case CMD_SEEK_B:
                if (e->decks[1].sf) {
                    sf_seek(e->decks[1].sf, (sf_count_t)(cmd->value * e->decks[1].info.samplerate), SEEK_SET);
                    if (e->decks[1].st) g_st.flush(e->decks[1].st);
                    e->decks[1].buf_index = 0;
                    e->decks[1].finished = 0;
                    e->decks[1].position = cmd->value;
                    e->decks[1].last_pos_update = now_sec();
                    start_deck(&e->decks[1]);
                }
                break;
            case CMD_SET_CROSSFADER:
                set_crossfader(e, cmd->value);
                break;
            case CMD_SET_TARGET_BPM:
                e->target_bpm = (float)cmd->value;
                for (int i = 0; i < 2; ++i) apply_tempo(&e->decks[i], e->target_bpm, e->beatmatch);
                break;
            case CMD_SET_BEATMATCH:
                e->beatmatch = (int)cmd->value;
                for (int i = 0; i < 2; ++i) apply_tempo(&e->decks[i], e->target_bpm, e->beatmatch);
                break;
            case CMD_SYNC_NOW:
                for (int i = 0; i < 2; ++i) apply_tempo(&e->decks[i], e->target_bpm, e->beatmatch);
                break;
            case CMD_AUTO_XFADE:
                e->auto_xfade_start = now_sec();
                e->auto_xfade_dur = cmd->value;
                e->auto_xfade_active = 1;
                break;
            case CMD_TEMPO_A:
                e->decks[0].tempo_mul = (float)cmd->value;
                apply_tempo(&e->decks[0], e->target_bpm, e->beatmatch);
                break;
            case CMD_TEMPO_B:
                e->decks[1].tempo_mul = (float)cmd->value;
                apply_tempo(&e->decks[1], e->target_bpm, e->beatmatch);
                break;
            case CMD_PITCH_A:
                e->decks[0].pitch_mul = (float)cmd->value;
                apply_tempo(&e->decks[0], e->target_bpm, e->beatmatch);
                break;
            case CMD_PITCH_B:
                e->decks[1].pitch_mul = (float)cmd->value;
                apply_tempo(&e->decks[1], e->target_bpm, e->beatmatch);
                break;
            case CMD_EQ_LOW_A: e->decks[0].eq_low = (float)cmd->value; break;
            case CMD_EQ_MID_A: e->decks[0].eq_mid = (float)cmd->value; break;
            case CMD_EQ_HIGH_A: e->decks[0].eq_high = (float)cmd->value; break;
            case CMD_EQ_LOW_B: e->decks[1].eq_low = (float)cmd->value; break;
            case CMD_EQ_MID_B: e->decks[1].eq_mid = (float)cmd->value; break;
            case CMD_EQ_HIGH_B: e->decks[1].eq_high = (float)cmd->value; break;
            case CMD_PLAY_SAMPLE: play_sample_tone(e); break;
            case CMD_GAIN_A: e->decks[0].gain_mul = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_GAIN_B: e->decks[1].gain_mul = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_MASTER_GAIN: e->master_gain = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_QUIT:
                g_free(cmd);
                return NULL;
            }
            g_free(cmd);
        }
        update_deck(&e->decks[0]);
        update_deck(&e->decks[1]);

        if (e->auto_xfade_active) {
            double t = (now_sec() - e->auto_xfade_start) / e->auto_xfade_dur;
            if (t >= 1.0) { t = 1.0; e->auto_xfade_active = 0; alSourceStop(e->decks[0].source); }
            set_crossfader(e, t);
        }

        double now = now_sec();
        double dt = now - last;
        if (dt < 0.02) g_usleep((useconds_t)((0.02 - dt) * 1e6));
        last = now;
    }
    return NULL;
}

static void send_cmd(EngineState *e, CommandType t, double v) {
    Command *c = g_new(Command, 1);
    c->type = t; c->value = v;
    g_async_queue_push(e->queue, c);
}

static void on_play_a(GtkButton *b, gpointer user_data) {
    (void)b;
    UI *ui = user_data;
    EngineState *e = g_object_get_data(G_OBJECT(ui->window), "engine");
    load_and_play_selection(e, GTK_LIST_BOX(ui->list_a), 0);
}
static void on_play_b(GtkButton *b, gpointer user_data) {
    (void)b;
    UI *ui = user_data;
    EngineState *e = g_object_get_data(G_OBJECT(ui->window), "engine");
    load_and_play_selection(e, GTK_LIST_BOX(ui->list_b), 1);
}
static void on_pp_a(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_PLAY_PAUSE_A, 0); }
static void on_pp_b(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_PLAY_PAUSE_B, 0); }
static void on_stop_a(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_STOP_A, 0); }
static void on_stop_b(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_STOP_B, 0); }
static void on_crossfader(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_CROSSFADER, gtk_range_get_value(r)); }
static void on_bpm(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_TARGET_BPM, gtk_range_get_value(r)); }
static void on_beatmatch(GtkToggleButton *t, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_BEATMATCH, gtk_toggle_button_get_active(t)); send_cmd(e, CMD_SYNC_NOW, e->target_bpm); }
static void on_sync(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_SYNC_NOW, e->target_bpm); }
static void on_sample(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_PLAY_SAMPLE, 0); }
static void on_seek_a(GtkRange *r, gpointer user_data) {
    EngineState *e = user_data;
    if (g_updating_seek) return;
    send_cmd(e, CMD_SEEK_A, gtk_range_get_value(r));
}
static void on_seek_b(GtkRange *r, gpointer user_data) {
    EngineState *e = user_data;
    if (g_updating_seek) return;
    send_cmd(e, CMD_SEEK_B, gtk_range_get_value(r));
}
static void on_tempo_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_TEMPO_A, gtk_range_get_value(r)); }
static void on_tempo_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_TEMPO_B, gtk_range_get_value(r)); }
static void on_pitch_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_PITCH_A, gtk_range_get_value(r)); }
static void on_pitch_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_PITCH_B, gtk_range_get_value(r)); }
static void on_eq_low_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_LOW_A, gtk_range_get_value(r)); }
static void on_eq_mid_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_MID_A, gtk_range_get_value(r)); }
static void on_eq_high_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_HIGH_A, gtk_range_get_value(r)); }
static void on_eq_low_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_LOW_B, gtk_range_get_value(r)); }
static void on_eq_mid_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_MID_B, gtk_range_get_value(r)); }
static void on_eq_high_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_EQ_HIGH_B, gtk_range_get_value(r)); }
static void on_gain_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_GAIN_A, gtk_range_get_value(r)); }
static void on_gain_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_GAIN_B, gtk_range_get_value(r)); }
static void on_master_gain(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_MASTER_GAIN, gtk_range_get_value(r)); }
static void on_auto_xfade(GtkButton *b, gpointer user_data) {
    (void)b;
    UI *ui = user_data;
    EngineState *e = g_object_get_data(G_OBJECT(ui->window), "engine");
    double v = gtk_range_get_value(GTK_RANGE(ui->xfade_time_scale));
    send_cmd(e, CMD_AUTO_XFADE, v);
}

static gboolean update_seekbars(gpointer data) {
    UiCtx *ctx = data;
    EngineState *e = ctx->eng;
    UI *ui = ctx->ui;
    g_updating_seek = TRUE;
    double pa = e->decks[0].position;
    double pb = e->decks[1].position;
    if (e->decks[0].duration > 0) {
        gtk_range_set_range(GTK_RANGE(ui->seek_a), 0.0, e->decks[0].duration);
        gtk_range_set_value(GTK_RANGE(ui->seek_a), pa);
    }
    if (e->decks[1].duration > 0) {
        gtk_range_set_range(GTK_RANGE(ui->seek_b), 0.0, e->decks[1].duration);
        gtk_range_set_value(GTK_RANGE(ui->seek_b), pb);
    }
    int ma = (int)(pa / 60), sa = (int)pa % 60;
    int mb = (int)(pb / 60), sb = (int)pb % 60;
    char buf[64];
    snprintf(buf, sizeof(buf), "A %02d:%02d / %02d:%02d",
             ma, sa,
             (int)(e->decks[0].duration / 60), (int)((int)e->decks[0].duration % 60));
    gtk_label_set_text(GTK_LABEL(ui->pos_label_a), buf);
    snprintf(buf, sizeof(buf), "B %02d:%02d / %02d:%02d",
             mb, sb,
             (int)(e->decks[1].duration / 60), (int)((int)e->decks[1].duration % 60));
    gtk_label_set_text(GTK_LABEL(ui->pos_label_b), buf);
    g_updating_seek = FALSE;
    return TRUE;
}

static void apply_css(void) {
    const char *css =
        "window { background: #222; color: #ddd; }\n"
        "scale slider { background: #444; border-radius: 4px; }\n"
        "button { background: #333; color: #eee; }\n";
    GtkCssProvider *prov = gtk_css_provider_new();
    gtk_css_provider_load_from_data(prov, css, -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(prov), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(prov);
}

static void build_ui(UI *ui, EngineState *e) {
    apply_css();
    ui->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(ui->window), "crossroads");
    gtk_window_set_default_size(GTK_WINDOW(ui->window), 1080, 720);
    g_signal_connect(ui->window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_object_set_data(G_OBJECT(ui->window), "engine", e);

    GtkWidget *grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(grid), 8);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 10);
    gtk_container_add(GTK_CONTAINER(ui->window), grid);

    GtkWidget *scroll_a = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll_a), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll_a, 260, -1);
    ui->list_a = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(ui->list_a), GTK_SELECTION_SINGLE);
    for (size_t i = 0; i < e->playlist.count; ++i) {
        GtkWidget *row = gtk_label_new(e->playlist.items[i]);
        gtk_widget_set_halign(row, GTK_ALIGN_START);
        gtk_label_set_ellipsize(GTK_LABEL(row), PANGO_ELLIPSIZE_MIDDLE);
        gtk_list_box_insert(GTK_LIST_BOX(ui->list_a), row, -1);
    }
    gtk_list_box_select_row(GTK_LIST_BOX(ui->list_a), gtk_list_box_get_row_at_index(GTK_LIST_BOX(ui->list_a), 0));
    gtk_container_add(GTK_CONTAINER(scroll_a), ui->list_a);
    gtk_grid_attach(GTK_GRID(grid), scroll_a, 0, 0, 1, 10);

    GtkWidget *scroll_b = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll_b), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll_b, 260, -1);
    ui->list_b = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(ui->list_b), GTK_SELECTION_SINGLE);
    for (size_t i = 0; i < e->playlist.count; ++i) {
        GtkWidget *row = gtk_label_new(e->playlist.items[i]);
        gtk_widget_set_halign(row, GTK_ALIGN_START);
        gtk_label_set_ellipsize(GTK_LABEL(row), PANGO_ELLIPSIZE_MIDDLE);
        gtk_list_box_insert(GTK_LIST_BOX(ui->list_b), row, -1);
    }
    gtk_list_box_select_row(GTK_LIST_BOX(ui->list_b), gtk_list_box_get_row_at_index(GTK_LIST_BOX(ui->list_b), 0));
    gtk_container_add(GTK_CONTAINER(scroll_b), ui->list_b);
    gtk_grid_attach(GTK_GRID(grid), scroll_b, 3, 0, 1, 10);

    ui->play_a = gtk_button_new_with_label("Play Deck A");
    g_signal_connect(ui->play_a, "clicked", G_CALLBACK(on_play_a), ui);
    gtk_grid_attach(GTK_GRID(grid), ui->play_a, 0, 10, 1, 1);

    ui->play_b = gtk_button_new_with_label("Play Deck B");
    g_signal_connect(ui->play_b, "clicked", G_CALLBACK(on_play_b), ui);
    gtk_grid_attach(GTK_GRID(grid), ui->play_b, 3, 10, 1, 1);

    ui->pp_a = gtk_button_new_with_label("Play/Pause A");
    g_signal_connect(ui->pp_a, "clicked", G_CALLBACK(on_pp_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->pp_a, 0, 11, 1, 1);
    ui->pp_b = gtk_button_new_with_label("Play/Pause B");
    g_signal_connect(ui->pp_b, "clicked", G_CALLBACK(on_pp_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->pp_b, 3, 11, 1, 1);

    ui->stop_a = gtk_button_new_with_label("Stop A");
    g_signal_connect(ui->stop_a, "clicked", G_CALLBACK(on_stop_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->stop_a, 0, 12, 1, 1);
    ui->stop_b = gtk_button_new_with_label("Stop B");
    g_signal_connect(ui->stop_b, "clicked", G_CALLBACK(on_stop_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->stop_b, 3, 12, 1, 1);

    ui->seek_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 600.0, 1.0);
    gtk_scale_set_value_pos(GTK_SCALE(ui->seek_a), GTK_POS_RIGHT);
    g_signal_connect(ui->seek_a, "value-changed", G_CALLBACK(on_seek_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->seek_a, 0, 13, 1, 1);
    ui->pos_label_a = gtk_label_new("A 00:00 / 00:00");
    gtk_grid_attach(GTK_GRID(grid), ui->pos_label_a, 0, 14, 1, 1);

    ui->seek_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 600.0, 1.0);
    gtk_scale_set_value_pos(GTK_SCALE(ui->seek_b), GTK_POS_RIGHT);
    g_signal_connect(ui->seek_b, "value-changed", G_CALLBACK(on_seek_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->seek_b, 3, 13, 1, 1);
    ui->pos_label_b = gtk_label_new("B 00:00 / 00:00");
    gtk_grid_attach(GTK_GRID(grid), ui->pos_label_b, 3, 14, 1, 1);

    GtkWidget *cross_label = gtk_label_new("Crossfader");
    gtk_grid_attach(GTK_GRID(grid), cross_label, 1, 0, 1, 1);
    ui->crossfader = gtk_scale_new_with_range(GTK_ORIENTATION_VERTICAL, 0.0, 1.0, 0.01);
    gtk_range_set_inverted(GTK_RANGE(ui->crossfader), TRUE);
    gtk_range_set_value(GTK_RANGE(ui->crossfader), 0.5);
    g_signal_connect(ui->crossfader, "value-changed", G_CALLBACK(on_crossfader), e);
    gtk_grid_attach(GTK_GRID(grid), ui->crossfader, 1, 1, 1, 8);

    GtkWidget *bpm_label = gtk_label_new("Target BPM");
    gtk_grid_attach(GTK_GRID(grid), bpm_label, 2, 0, 1, 1);
    ui->bpm_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 60.0, 200.0, 1.0);
    gtk_scale_set_value_pos(GTK_SCALE(ui->bpm_scale), GTK_POS_RIGHT);
    gtk_range_set_value(GTK_RANGE(ui->bpm_scale), e->target_bpm);
    g_signal_connect(ui->bpm_scale, "value-changed", G_CALLBACK(on_bpm), e);
    gtk_grid_attach(GTK_GRID(grid), ui->bpm_scale, 2, 1, 1, 1);

    ui->beatmatch_toggle = gtk_check_button_new_with_label("Beatmatch (Pitch-Lock)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->beatmatch_toggle), FALSE);
    g_signal_connect(ui->beatmatch_toggle, "toggled", G_CALLBACK(on_beatmatch), e);
    gtk_grid_attach(GTK_GRID(grid), ui->beatmatch_toggle, 2, 2, 1, 1);

    ui->sync_button = gtk_button_new_with_label("Sync to BPM");
    g_signal_connect(ui->sync_button, "clicked", G_CALLBACK(on_sync), e);
    gtk_grid_attach(GTK_GRID(grid), ui->sync_button, 2, 3, 1, 1);

    GtkWidget *master_label = gtk_label_new("Master Gain");
    gtk_grid_attach(GTK_GRID(grid), master_label, 2, 4, 1, 1);
    ui->master_gain = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->master_gain), e->master_gain);
    g_signal_connect(ui->master_gain, "value-changed", G_CALLBACK(on_master_gain), e);
    gtk_grid_attach(GTK_GRID(grid), ui->master_gain, 2, 5, 1, 1);

    GtkWidget *xfade_time_label = gtk_label_new("Auto Crossfade (s)");
    gtk_grid_attach(GTK_GRID(grid), xfade_time_label, 2, 6, 1, 1);
    ui->xfade_time_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 2.0, 20.0, 0.5);
    gtk_range_set_value(GTK_RANGE(ui->xfade_time_scale), 8.0);
    gtk_grid_attach(GTK_GRID(grid), ui->xfade_time_scale, 2, 7, 1, 1);
    ui->auto_xfade_btn = gtk_button_new_with_label("Auto Crossfade");
    g_signal_connect(ui->auto_xfade_btn, "clicked", G_CALLBACK(on_auto_xfade), ui);
    gtk_grid_attach(GTK_GRID(grid), ui->auto_xfade_btn, 2, 8, 1, 1);

    GtkWidget *gain_label_a = gtk_label_new("Gain Deck A");
    gtk_grid_attach(GTK_GRID(grid), gain_label_a, 0, 15, 1, 1);
    ui->gain_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->gain_a), 1.0);
    g_signal_connect(ui->gain_a, "value-changed", G_CALLBACK(on_gain_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->gain_a, 0, 16, 1, 1);

    GtkWidget *gain_label_b = gtk_label_new("Gain Deck B");
    gtk_grid_attach(GTK_GRID(grid), gain_label_b, 3, 15, 1, 1);
    ui->gain_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->gain_b), 1.0);
    g_signal_connect(ui->gain_b, "value-changed", G_CALLBACK(on_gain_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->gain_b, 3, 16, 1, 1);

    GtkWidget *tempo_label_a = gtk_label_new("Tempo Deck A");
    gtk_grid_attach(GTK_GRID(grid), tempo_label_a, 0, 17, 1, 1);
    ui->tempo_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(ui->tempo_a), 1.0);
    g_signal_connect(ui->tempo_a, "value-changed", G_CALLBACK(on_tempo_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->tempo_a, 0, 18, 1, 1);

    GtkWidget *tempo_label_b = gtk_label_new("Tempo Deck B");
    gtk_grid_attach(GTK_GRID(grid), tempo_label_b, 3, 17, 1, 1);
    ui->tempo_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(ui->tempo_b), 1.0);
    g_signal_connect(ui->tempo_b, "value-changed", G_CALLBACK(on_tempo_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->tempo_b, 3, 18, 1, 1);

    GtkWidget *pitch_label_a = gtk_label_new("Pitch Deck A");
    gtk_grid_attach(GTK_GRID(grid), pitch_label_a, 0, 19, 1, 1);
    ui->pitch_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(ui->pitch_a), 1.0);
    g_signal_connect(ui->pitch_a, "value-changed", G_CALLBACK(on_pitch_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->pitch_a, 0, 20, 1, 1);

    GtkWidget *pitch_label_b = gtk_label_new("Pitch Deck B");
    gtk_grid_attach(GTK_GRID(grid), pitch_label_b, 3, 19, 1, 1);
    ui->pitch_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(ui->pitch_b), 1.0);
    g_signal_connect(ui->pitch_b, "value-changed", G_CALLBACK(on_pitch_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->pitch_b, 3, 20, 1, 1);

    GtkWidget *eq_label_a = gtk_label_new("EQ Deck A (Low/Mid/High)");
    gtk_grid_attach(GTK_GRID(grid), eq_label_a, 0, 21, 1, 1);
    ui->eq_low_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_low_a), 1.0);
    g_signal_connect(ui->eq_low_a, "value-changed", G_CALLBACK(on_eq_low_a), e);
    ui->eq_mid_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_mid_a), 1.0);
    g_signal_connect(ui->eq_mid_a, "value-changed", G_CALLBACK(on_eq_mid_a), e);
    ui->eq_high_a = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_high_a), 1.0);
    g_signal_connect(ui->eq_high_a, "value-changed", G_CALLBACK(on_eq_high_a), e);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_low_a, 0, 22, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_mid_a, 0, 23, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_high_a, 0, 24, 1, 1);

    GtkWidget *eq_label_b = gtk_label_new("EQ Deck B (Low/Mid/High)");
    gtk_grid_attach(GTK_GRID(grid), eq_label_b, 3, 21, 1, 1);
    ui->eq_low_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_low_b), 1.0);
    g_signal_connect(ui->eq_low_b, "value-changed", G_CALLBACK(on_eq_low_b), e);
    ui->eq_mid_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_mid_b), 1.0);
    g_signal_connect(ui->eq_mid_b, "value-changed", G_CALLBACK(on_eq_mid_b), e);
    ui->eq_high_b = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->eq_high_b), 1.0);
    g_signal_connect(ui->eq_high_b, "value-changed", G_CALLBACK(on_eq_high_b), e);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_low_b, 3, 22, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_mid_b, 3, 23, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), ui->eq_high_b, 3, 24, 1, 1);

    ui->sample_btn = gtk_button_new_with_label("Play Sample");
    g_signal_connect(ui->sample_btn, "clicked", G_CALLBACK(on_sample), e);
    gtk_grid_attach(GTK_GRID(grid), ui->sample_btn, 2, 9, 1, 1);

    gtk_widget_show_all(ui->window);
}

static int load_playlist_dir(const char *dirpath, Playlist *pl, char *err, size_t errsz) {
    memset(pl, 0, sizeof(*pl));
    DIR *d = opendir(dirpath);
    if (!d) { snprintf(err, errsz, "Cannot open %s", dirpath); return -1; }
    struct dirent *ent;
    while ((ent = readdir(d))) {
        const char *name = ent->d_name;
        if (name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0'))) continue;
        size_t len = strlen(name);
        if (len < 4) continue;
        const char *ext = name + len - 4;
        if (strcasecmp(ext, ".mp3") != 0) continue;
        char full[PATH_MAX];
        snprintf(full, sizeof(full), "%s/%s", dirpath, name);
        pl->items = realloc(pl->items, (pl->count + 1) * sizeof(char *));
        pl->items[pl->count++] = strdup(full);
    }
    closedir(d);
    if (!pl->count) { snprintf(err, errsz, "No mp3 files in %s", dirpath); return -1; }
    qsort(pl->items, pl->count, sizeof(char *), (int (*)(const void *, const void *))strcmp);
    return 0;
}

static void playlist_free(Playlist *pl) {
    for (size_t i = 0; i < pl->count; ++i) free(pl->items[i]);
    free(pl->items);
}

static void sig_handler(int s) { (void)s; g_should_quit = 1; }

int main(int argc, char **argv) {
    const char *dir = (argc >= 2) ? argv[1] : "mp3_library";
    signal(SIGINT, sig_handler);

    EngineState eng;
    memset(&eng, 0, sizeof(eng));
    eng.crossfader = 0.5;
    eng.target_bpm = 128.f;
    eng.beatmatch = 0;
    eng.master_gain = 1.0f;
    char err[256];
    if (load_playlist_dir(dir, &eng.playlist, err, sizeof(err)) != 0) {
        fprintf(stderr, "%s\n", err);
        return 1;
    }

    eng.dev = alcOpenDevice(NULL);
    if (!eng.dev) { fprintf(stderr, "OpenAL device failed\n"); return 1; }
    eng.ctx = alcCreateContext(eng.dev, NULL);
    if (!eng.ctx || !alcMakeContextCurrent(eng.ctx)) { fprintf(stderr, "OpenAL context failed\n"); return 1; }

    gtk_init(&argc, &argv);
    eng.queue = g_async_queue_new();

    if (eng.playlist.count > 0) {
        init_deck(&eng.decks[0], eng.playlist.items[0], eng.target_bpm, eng.beatmatch, err, sizeof(err));
        init_deck(&eng.decks[1], eng.playlist.items[0], eng.target_bpm, eng.beatmatch, err, sizeof(err));
        set_crossfader(&eng, 0.5);
    }

    UI ui;
    memset(&ui, 0, sizeof(ui));
    build_ui(&ui, &eng);

    UiCtx ctx = {.eng = &eng, .ui = &ui};
    g_timeout_add(200, update_seekbars, &ctx);

    g_thread_new("engine", engine_thread, &eng);
    gtk_main();

    Command *q = g_new(Command, 1);
    q->type = CMD_QUIT; q->value = 0;
    g_async_queue_push(eng.queue, q);
    g_usleep(200000);

    destroy_deck(&eng.decks[0]);
    destroy_deck(&eng.decks[1]);
    playlist_free(&eng.playlist);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(eng.ctx);
    alcCloseDevice(eng.dev);
    return 0;
}
