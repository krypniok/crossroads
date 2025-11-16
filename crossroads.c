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
    CMD_EQ_BAND_A,
    CMD_EQ_BAND_B,
    CMD_EQ_BAND_MASTER,
    CMD_SET_SAMPLE_DURATION,
    CMD_SET_SAMPLE_GAIN,
    CMD_QUIT
} CommandType;

typedef struct {
    CommandType type;
    double value;
    int index;
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
    GtkWidget *gain_a;
    GtkWidget *gain_b;
    GtkWidget *master_gain;
    GtkWidget *pos_label_a;
    GtkWidget *pos_label_b;
    GtkWidget *eq_bands_a[10];
    GtkWidget *eq_bands_b[10];
    GtkWidget *eq_bands_master[10];
    GtkWidget *sample_preset;
    GtkWidget *sample_duration;
    GtkWidget *sample_gain;
    GtkWidget *sample_play;
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
    float deck_eq_bands[2][10];
    float master_eq_bands[10];
    double sample_duration;
    float sample_gain;
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

static void bands_to_lmh(const float bands[10], float *low, float *mid, float *high) {
    /* Use a squared average so single bands have stronger impact. */
    float l = 0.f, m = 0.f, h = 0.f;
    for (int i = 0; i < 3; ++i) l += bands[i];
    for (int i = 3; i < 7; ++i) m += bands[i];
    for (int i = 7; i < 10; ++i) h += bands[i];
    l = l / 3.f; m = m / 4.f; h = h / 3.f;
    l = l * l * 1.5f;
    m = m * m * 1.5f;
    h = h * h * 1.5f;
    if (l > 4.f) l = 4.f;
    if (m > 4.f) m = 4.f;
    if (h > 4.f) h = 4.f;
    *low = l > 0.f ? l : 0.f;
    *mid = m > 0.f ? m : 0.f;
    *high = h > 0.f ? h : 0.f;
}

static void refresh_eq_for_deck(EngineState *e, int deck_id) {
    if (deck_id < 0 || deck_id > 1) return;
    float dl = 1.f, dm = 1.f, dh = 1.f;
    float ml = 1.f, mm = 1.f, mh = 1.f;
    bands_to_lmh(e->deck_eq_bands[deck_id], &dl, &dm, &dh);
    bands_to_lmh(e->master_eq_bands, &ml, &mm, &mh);
    e->decks[deck_id].eq_low = dl * ml;
    e->decks[deck_id].eq_mid = dm * mm;
    e->decks[deck_id].eq_high = dh * mh;
}

static void refresh_all_eq(EngineState *e) {
    refresh_eq_for_deck(e, 0);
    refresh_eq_for_deck(e, 1);
}

typedef struct {
    double freq;
    double dur;
    float gain;
} SampleParams;

static gpointer sample_thread(gpointer data) {
    SampleParams *p = data;
    const int sr = 44100;
    const double freq = (p && p->freq > 0.0) ? p->freq : 440.0;
    const double dur = (p && p->dur > 0.05) ? p->dur : 1.0;
    const float gain = (p && p->gain > 0.f) ? p->gain : 0.6f;
    size_t frames = (size_t)(sr * dur);
    int16_t *pcm = malloc(frames * 2 * sizeof(int16_t));
    if (!pcm) { g_free(p); return NULL; }
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
    alSourcef(src, AL_GAIN, gain);
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
    g_free(p);
    return NULL;
}

static void play_sample_tone(EngineState *e, double freq) {
    SampleParams *p = g_new0(SampleParams, 1);
    if (!p) return;
    p->freq = freq > 0.0 ? freq : 440.0;
    p->dur = e->sample_duration > 0.05 ? e->sample_duration : 0.05;
    p->gain = e->sample_gain > 0.f ? e->sample_gain : 0.6f;
    g_thread_new("sample", sample_thread, p);
}

static void load_and_play_selection(EngineState *e, GtkListBox *list, int deck_id) {
    GtkListBoxRow *row = gtk_list_box_get_selected_row(list);
    if (!row) return;
    int idx = gtk_list_box_row_get_index(row);
    if (idx < 0 || (size_t)idx >= e->playlist.count) return;
    char err[256];
    if (init_deck(&e->decks[deck_id], e->playlist.items[idx], e->target_bpm, e->beatmatch, err, sizeof(err)) == 0) {
        apply_tempo(&e->decks[deck_id], e->target_bpm, e->beatmatch);
        refresh_eq_for_deck(e, deck_id);
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
            case CMD_EQ_LOW_A:
                for (int i = 0; i < 3; ++i) e->deck_eq_bands[0][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 0);
                break;
            case CMD_EQ_MID_A:
                for (int i = 3; i < 7; ++i) e->deck_eq_bands[0][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 0);
                break;
            case CMD_EQ_HIGH_A:
                for (int i = 7; i < 10; ++i) e->deck_eq_bands[0][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 0);
                break;
            case CMD_EQ_LOW_B:
                for (int i = 0; i < 3; ++i) e->deck_eq_bands[1][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 1);
                break;
            case CMD_EQ_MID_B:
                for (int i = 3; i < 7; ++i) e->deck_eq_bands[1][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 1);
                break;
            case CMD_EQ_HIGH_B:
                for (int i = 7; i < 10; ++i) e->deck_eq_bands[1][i] = (float)cmd->value;
                refresh_eq_for_deck(e, 1);
                break;
            case CMD_PLAY_SAMPLE: play_sample_tone(e, cmd->value); break;
            case CMD_GAIN_A: e->decks[0].gain_mul = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_GAIN_B: e->decks[1].gain_mul = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_MASTER_GAIN: e->master_gain = (float)cmd->value; set_crossfader(e, e->crossfader); break;
            case CMD_EQ_BAND_A:
                if (cmd->index >= 0 && cmd->index < 10) { e->deck_eq_bands[0][cmd->index] = (float)cmd->value; refresh_eq_for_deck(e, 0); }
                break;
            case CMD_EQ_BAND_B:
                if (cmd->index >= 0 && cmd->index < 10) { e->deck_eq_bands[1][cmd->index] = (float)cmd->value; refresh_eq_for_deck(e, 1); }
                break;
            case CMD_EQ_BAND_MASTER:
                if (cmd->index >= 0 && cmd->index < 10) { e->master_eq_bands[cmd->index] = (float)cmd->value; refresh_all_eq(e); }
                break;
            case CMD_SET_SAMPLE_DURATION:
                e->sample_duration = cmd->value > 0.05 ? cmd->value : 0.05;
                break;
            case CMD_SET_SAMPLE_GAIN:
                if (cmd->value < 0.0) cmd->value = 0.0;
                if (cmd->value > 2.0) cmd->value = 2.0;
                e->sample_gain = (float)cmd->value;
                break;
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

static void send_cmd(EngineState *e, CommandType t, double v, int idx) {
    Command *c = g_new(Command, 1);
    c->type = t; c->value = v; c->index = idx;
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
static void on_pp_a(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_PLAY_PAUSE_A, 0, -1); }
static void on_pp_b(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_PLAY_PAUSE_B, 0, -1); }
static void on_stop_a(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_STOP_A, 0, -1); }
static void on_stop_b(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_STOP_B, 0, -1); }
static void on_crossfader(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_CROSSFADER, gtk_range_get_value(r), -1); }
static void on_bpm(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_TARGET_BPM, gtk_range_get_value(r), -1); }
static void on_beatmatch(GtkToggleButton *t, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_BEATMATCH, gtk_toggle_button_get_active(t), -1); send_cmd(e, CMD_SYNC_NOW, e->target_bpm, -1); }
static void on_sync(GtkButton *b, gpointer user_data) { (void)b; EngineState *e = user_data; send_cmd(e, CMD_SYNC_NOW, e->target_bpm, -1); }
static void on_seek_a(GtkRange *r, gpointer user_data) {
    EngineState *e = user_data;
    if (g_updating_seek) return;
    send_cmd(e, CMD_SEEK_A, gtk_range_get_value(r), -1);
}
static void on_seek_b(GtkRange *r, gpointer user_data) {
    EngineState *e = user_data;
    if (g_updating_seek) return;
    send_cmd(e, CMD_SEEK_B, gtk_range_get_value(r), -1);
}
static void on_tempo_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_TEMPO_A, gtk_range_get_value(r), -1); }
static void on_tempo_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_TEMPO_B, gtk_range_get_value(r), -1); }
static void on_pitch_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_PITCH_A, gtk_range_get_value(r), -1); }
static void on_pitch_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_PITCH_B, gtk_range_get_value(r), -1); }
static void on_gain_a(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_GAIN_A, gtk_range_get_value(r), -1); }
static void on_gain_b(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_GAIN_B, gtk_range_get_value(r), -1); }
static void on_master_gain(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_MASTER_GAIN, gtk_range_get_value(r), -1); }
typedef struct {
    EngineState *eng;
    CommandType cmd;
    int band;
} EqBind;
static void free_eq_bind(gpointer data, GClosure *closure) { (void)closure; g_free(data); }
static void on_eq_band(GtkRange *r, gpointer user_data) {
    EqBind *b = user_data;
    if (!b) return;
    send_cmd(b->eng, b->cmd, gtk_range_get_value(r), b->band);
}
static void on_sample_duration(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_SAMPLE_DURATION, gtk_range_get_value(r), -1); }
static void on_sample_gain(GtkRange *r, gpointer user_data) { EngineState *e = user_data; send_cmd(e, CMD_SET_SAMPLE_GAIN, gtk_range_get_value(r), -1); }
static double get_selected_sample_freq(UI *ui) {
    if (!ui || !ui->sample_preset) return 440.0;
    const char *id = gtk_combo_box_get_active_id(GTK_COMBO_BOX(ui->sample_preset));
    if (!id) return 440.0;
    char *end = NULL;
    double v = g_ascii_strtod(id, &end);
    if (end == id || v <= 0.0) return 440.0;
    return v;
}
static void on_sample_play(GtkButton *b, gpointer user_data) {
    (void)b;
    UI *ui = user_data;
    EngineState *e = g_object_get_data(G_OBJECT(ui->window), "engine");
    double freq = get_selected_sample_freq(ui);
    send_cmd(e, CMD_PLAY_SAMPLE, freq, -1);
}
static void on_auto_xfade(GtkButton *b, gpointer user_data) {
    (void)b;
    UI *ui = user_data;
    EngineState *e = g_object_get_data(G_OBJECT(ui->window), "engine");
    double v = gtk_range_get_value(GTK_RANGE(ui->xfade_time_scale));
    send_cmd(e, CMD_AUTO_XFADE, v, -1);
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

static GtkWidget *create_eq_section(UI *ui, EngineState *e, const char *title, GtkWidget **slots, CommandType cmd, GtkAlign align) {
    (void)ui;
    static const char *freqs[10] = {"31", "62", "125", "250", "500", "1k", "2k", "4k", "8k", "16k"};
    GtkWidget *outer = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *lbl = gtk_label_new(title);
    gtk_widget_set_halign(lbl, align);
    gtk_box_pack_start(GTK_BOX(outer), lbl, FALSE, FALSE, 0);

    GtkWidget *bands_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_set_halign(bands_box, align);
    for (int i = 0; i < 10; ++i) {
        GtkWidget *band_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
        GtkWidget *scale = gtk_scale_new_with_range(GTK_ORIENTATION_VERTICAL, 0.0, 2.0, 0.05);
        gtk_scale_set_draw_value(GTK_SCALE(scale), FALSE);
        float init = 1.0f;
        if (cmd == CMD_EQ_BAND_A) init = e->deck_eq_bands[0][i];
        else if (cmd == CMD_EQ_BAND_B) init = e->deck_eq_bands[1][i];
        else if (cmd == CMD_EQ_BAND_MASTER) init = e->master_eq_bands[i];
        gtk_range_set_value(GTK_RANGE(scale), init);
        gtk_widget_set_size_request(scale, 32, 140);
        gtk_widget_set_halign(scale, GTK_ALIGN_CENTER);
        EqBind *bind = g_new(EqBind, 1);
        bind->eng = e; bind->cmd = cmd; bind->band = i;
        g_signal_connect_data(scale, "value-changed", G_CALLBACK(on_eq_band), bind, free_eq_bind, 0);
        if (slots) slots[i] = scale;
        GtkWidget *flabel = gtk_label_new(freqs[i]);
        gtk_widget_set_halign(flabel, GTK_ALIGN_CENTER);
        gtk_box_pack_start(GTK_BOX(band_box), scale, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(band_box), flabel, FALSE, FALSE, 0);
        gtk_box_pack_start(GTK_BOX(bands_box), band_box, TRUE, TRUE, 0);
    }
    gtk_box_pack_start(GTK_BOX(outer), bands_box, FALSE, FALSE, 0);
    return outer;
}

static GtkWidget *create_deck_column(UI *ui, EngineState *e, int deck_id, GtkAlign align) {
    GtkWidget *col = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    gtk_widget_set_halign(col, align);
    gtk_widget_set_valign(col, GTK_ALIGN_START);

    const char *deck_name = deck_id == 0 ? "Deck A" : "Deck B";
    GtkWidget *title = gtk_label_new(deck_name);
    gtk_widget_set_halign(title, align);
    gtk_box_pack_start(GTK_BOX(col), title, FALSE, FALSE, 0);

    GtkWidget **list_ptr = deck_id == 0 ? &ui->list_a : &ui->list_b;
    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll, 260, 250);
    *list_ptr = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(*list_ptr), GTK_SELECTION_SINGLE);
    for (size_t i = 0; i < e->playlist.count; ++i) {
        GtkWidget *row = gtk_label_new(e->playlist.items[i]);
        gtk_widget_set_halign(row, align);
        gtk_label_set_ellipsize(GTK_LABEL(row), PANGO_ELLIPSIZE_MIDDLE);
        gtk_list_box_insert(GTK_LIST_BOX(*list_ptr), row, -1);
    }
    gtk_list_box_select_row(GTK_LIST_BOX(*list_ptr), gtk_list_box_get_row_at_index(GTK_LIST_BOX(*list_ptr), 0));
    gtk_container_add(GTK_CONTAINER(scroll), *list_ptr);
    gtk_widget_set_halign(scroll, align);
    gtk_box_pack_start(GTK_BOX(col), scroll, TRUE, TRUE, 0);

    GtkWidget **pos_ptr = deck_id == 0 ? &ui->pos_label_a : &ui->pos_label_b;
    *pos_ptr = gtk_label_new(deck_id == 0 ? "A 00:00 / 00:00" : "B 00:00 / 00:00");
    gtk_widget_set_halign(*pos_ptr, align);
    gtk_box_pack_start(GTK_BOX(col), *pos_ptr, FALSE, FALSE, 0);

    GtkWidget **seek_ptr = deck_id == 0 ? &ui->seek_a : &ui->seek_b;
    *seek_ptr = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 600.0, 1.0);
    gtk_widget_set_hexpand(*seek_ptr, TRUE);
    gtk_widget_set_size_request(*seek_ptr, 260, -1);
    gtk_widget_set_halign(*seek_ptr, align);
    gtk_scale_set_value_pos(GTK_SCALE(*seek_ptr), align == GTK_ALIGN_END ? GTK_POS_LEFT : GTK_POS_RIGHT);
    g_signal_connect(*seek_ptr, "value-changed", G_CALLBACK(deck_id == 0 ? on_seek_a : on_seek_b), e);
    gtk_box_pack_start(GTK_BOX(col), *seek_ptr, FALSE, FALSE, 0);

    GtkWidget *transport = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_set_halign(transport, align);
    GtkWidget *play_btn = gtk_button_new_with_label(deck_id == 0 ? "Play Deck A" : "Play Deck B");
    gtk_box_pack_start(GTK_BOX(transport), play_btn, FALSE, FALSE, 0);
    if (deck_id == 0) { ui->play_a = play_btn; g_signal_connect(play_btn, "clicked", G_CALLBACK(on_play_a), ui); }
    else { ui->play_b = play_btn; g_signal_connect(play_btn, "clicked", G_CALLBACK(on_play_b), ui); }
    GtkWidget *pp = gtk_button_new_with_label(deck_id == 0 ? "Play/Pause A" : "Play/Pause B");
    gtk_box_pack_start(GTK_BOX(transport), pp, FALSE, FALSE, 0);
    if (deck_id == 0) { ui->pp_a = pp; g_signal_connect(pp, "clicked", G_CALLBACK(on_pp_a), e); }
    else { ui->pp_b = pp; g_signal_connect(pp, "clicked", G_CALLBACK(on_pp_b), e); }
    GtkWidget *stop = gtk_button_new_with_label(deck_id == 0 ? "Stop A" : "Stop B");
    gtk_box_pack_start(GTK_BOX(transport), stop, FALSE, FALSE, 0);
    if (deck_id == 0) { ui->stop_a = stop; g_signal_connect(stop, "clicked", G_CALLBACK(on_stop_a), e); }
    else { ui->stop_b = stop; g_signal_connect(stop, "clicked", G_CALLBACK(on_stop_b), e); }
    gtk_box_pack_start(GTK_BOX(col), transport, FALSE, FALSE, 0);

    GtkWidget *gain_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    GtkWidget *gain_label = gtk_label_new(deck_id == 0 ? "Gain A" : "Gain B");
    gtk_widget_set_halign(gain_label, align);
    gtk_box_pack_start(GTK_BOX(gain_box), gain_label, FALSE, FALSE, 0);
    GtkWidget **gain_ptr = deck_id == 0 ? &ui->gain_a : &ui->gain_b;
    *gain_ptr = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(*gain_ptr), 1.0);
    gtk_widget_set_hexpand(*gain_ptr, TRUE);
    gtk_widget_set_size_request(*gain_ptr, 260, -1);
    gtk_widget_set_halign(*gain_ptr, align);
    g_signal_connect(*gain_ptr, "value-changed", G_CALLBACK(deck_id == 0 ? on_gain_a : on_gain_b), e);
    gtk_box_pack_start(GTK_BOX(gain_box), *gain_ptr, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(col), gain_box, FALSE, FALSE, 0);

    GtkWidget *tempo_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    GtkWidget *tempo_label = gtk_label_new(deck_id == 0 ? "Tempo A" : "Tempo B");
    gtk_widget_set_halign(tempo_label, align);
    gtk_box_pack_start(GTK_BOX(tempo_box), tempo_label, FALSE, FALSE, 0);
    GtkWidget **tempo_ptr = deck_id == 0 ? &ui->tempo_a : &ui->tempo_b;
    *tempo_ptr = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(*tempo_ptr), 1.0);
    gtk_widget_set_hexpand(*tempo_ptr, TRUE);
    gtk_widget_set_size_request(*tempo_ptr, 260, -1);
    gtk_widget_set_halign(*tempo_ptr, align);
    g_signal_connect(*tempo_ptr, "value-changed", G_CALLBACK(deck_id == 0 ? on_tempo_a : on_tempo_b), e);
    gtk_box_pack_start(GTK_BOX(tempo_box), *tempo_ptr, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(col), tempo_box, FALSE, FALSE, 0);

    GtkWidget *pitch_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    GtkWidget *pitch_label = gtk_label_new(deck_id == 0 ? "Pitch A" : "Pitch B");
    gtk_widget_set_halign(pitch_label, align);
    gtk_box_pack_start(GTK_BOX(pitch_box), pitch_label, FALSE, FALSE, 0);
    GtkWidget **pitch_ptr = deck_id == 0 ? &ui->pitch_a : &ui->pitch_b;
    *pitch_ptr = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 2.0, 0.01);
    gtk_range_set_value(GTK_RANGE(*pitch_ptr), 1.0);
    gtk_widget_set_hexpand(*pitch_ptr, TRUE);
    gtk_widget_set_size_request(*pitch_ptr, 260, -1);
    gtk_widget_set_halign(*pitch_ptr, align);
    g_signal_connect(*pitch_ptr, "value-changed", G_CALLBACK(deck_id == 0 ? on_pitch_a : on_pitch_b), e);
    gtk_box_pack_start(GTK_BOX(pitch_box), *pitch_ptr, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(col), pitch_box, FALSE, FALSE, 0);

    GtkWidget *eq = create_eq_section(ui, e, deck_id == 0 ? "Deck A EQ (10-Band)" : "Deck B EQ (10-Band)",
                                      deck_id == 0 ? ui->eq_bands_a : ui->eq_bands_b,
                                      deck_id == 0 ? CMD_EQ_BAND_A : CMD_EQ_BAND_B,
                                      align);
    gtk_box_pack_start(GTK_BOX(col), eq, FALSE, FALSE, 0);

    return col;
}

static void apply_css(void) {
    const char *css =
        "window { background: #dcdcdc; color: #111; }\n"
        "label { color: #111; }\n"
        "button { background: #d0d0d0; color: #111; border: 1px solid #777; border-radius: 6px; }\n"
        "button:hover { background: #c4c4c4; }\n"
        "scale trough { background: #c2c2c2; border-radius: 6px; border: 1px solid #8a8a8a; }\n"
        "scale highlight { background: #ff9f1c; }\n"
        "scale slider { background: #f0ad2c; border: 1px solid #c97f00; border-radius: 8px; min-height: 20px; min-width: 20px; }\n"
        "scale value { color: #111; font-weight: bold; }\n"
        "treeview, list, list row, list row > * { color: #111; background: #ededed; }\n"
        "list row:selected, list row:selected > * { background: #4a80c2; color: #fff; }\n"
        "scrolledwindow { background: #ededed; border: 1px solid #999; }\n"
        ".notebook tab { background: #cfcfcf; color: #111; border: 1px solid #888; }\n";
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

    GtkWidget *notebook = gtk_notebook_new();
    gtk_container_add(GTK_CONTAINER(ui->window), notebook);

    GtkWidget *grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 12);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 18);

    GtkWidget *left_col = create_deck_column(ui, e, 0, GTK_ALIGN_START);
    GtkWidget *right_col = create_deck_column(ui, e, 1, GTK_ALIGN_END);

    GtkWidget *center_col = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_widget_set_halign(center_col, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(center_col, GTK_ALIGN_START);

    GtkWidget *mix_label = gtk_label_new("Mixer / Master");
    gtk_widget_set_halign(mix_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), mix_label, FALSE, FALSE, 0);

    GtkWidget *cross_label = gtk_label_new("Crossfader A ↔ B");
    gtk_widget_set_halign(cross_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), cross_label, FALSE, FALSE, 0);
    ui->crossfader = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 1.0, 0.01);
    gtk_range_set_value(GTK_RANGE(ui->crossfader), e->crossfader);
    gtk_widget_set_hexpand(ui->crossfader, TRUE);
    g_signal_connect(ui->crossfader, "value-changed", G_CALLBACK(on_crossfader), e);
    gtk_box_pack_start(GTK_BOX(center_col), ui->crossfader, FALSE, FALSE, 0);

    GtkWidget *bpm_label = gtk_label_new("Ziel-BPM");
    gtk_widget_set_halign(bpm_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), bpm_label, FALSE, FALSE, 0);
    ui->bpm_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 60.0, 200.0, 0.5);
    gtk_range_set_value(GTK_RANGE(ui->bpm_scale), e->target_bpm);
    gtk_widget_set_hexpand(ui->bpm_scale, TRUE);
    g_signal_connect(ui->bpm_scale, "value-changed", G_CALLBACK(on_bpm), e);
    gtk_box_pack_start(GTK_BOX(center_col), ui->bpm_scale, FALSE, FALSE, 0);

    ui->beatmatch_toggle = gtk_check_button_new_with_label("Beatmatch (Pitch-Lock)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->beatmatch_toggle), e->beatmatch != 0);
    gtk_widget_set_halign(ui->beatmatch_toggle, GTK_ALIGN_CENTER);
    g_signal_connect(ui->beatmatch_toggle, "toggled", G_CALLBACK(on_beatmatch), e);
    gtk_box_pack_start(GTK_BOX(center_col), ui->beatmatch_toggle, FALSE, FALSE, 0);

    ui->sync_button = gtk_button_new_with_label("Sync to BPM");
    gtk_widget_set_halign(ui->sync_button, GTK_ALIGN_CENTER);
    g_signal_connect(ui->sync_button, "clicked", G_CALLBACK(on_sync), e);
    gtk_box_pack_start(GTK_BOX(center_col), ui->sync_button, FALSE, FALSE, 0);

    GtkWidget *master_label = gtk_label_new("Master Gain");
    gtk_widget_set_halign(master_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), master_label, FALSE, FALSE, 0);
    ui->master_gain = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 2.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->master_gain), e->master_gain);
    gtk_widget_set_hexpand(ui->master_gain, TRUE);
    g_signal_connect(ui->master_gain, "value-changed", G_CALLBACK(on_master_gain), e);
    gtk_box_pack_start(GTK_BOX(center_col), ui->master_gain, FALSE, FALSE, 0);

    GtkWidget *xfade_time_label = gtk_label_new("Auto Crossfade (s)");
    gtk_widget_set_halign(xfade_time_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), xfade_time_label, FALSE, FALSE, 0);
    ui->xfade_time_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 2.0, 20.0, 0.5);
    gtk_range_set_value(GTK_RANGE(ui->xfade_time_scale), 8.0);
    gtk_widget_set_hexpand(ui->xfade_time_scale, TRUE);
    gtk_box_pack_start(GTK_BOX(center_col), ui->xfade_time_scale, FALSE, FALSE, 0);
    ui->auto_xfade_btn = gtk_button_new_with_label("Auto Crossfade");
    gtk_widget_set_halign(ui->auto_xfade_btn, GTK_ALIGN_CENTER);
    g_signal_connect(ui->auto_xfade_btn, "clicked", G_CALLBACK(on_auto_xfade), ui);
    gtk_box_pack_start(GTK_BOX(center_col), ui->auto_xfade_btn, FALSE, FALSE, 0);

    GtkWidget *master_eq = create_eq_section(ui, e, "Master EQ (10-Band)", ui->eq_bands_master, CMD_EQ_BAND_MASTER, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(center_col), master_eq, FALSE, FALSE, 0);

    gtk_grid_attach(GTK_GRID(grid), left_col, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), center_col, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), right_col, 2, 0, 1, 1);

    GtkWidget *sample_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(sample_box), 12);
    GtkWidget *sample_title = gtk_label_new("Sampler / Play Sample");
    gtk_widget_set_halign(sample_title, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(sample_box), sample_title, FALSE, FALSE, 0);

    ui->sample_preset = gtk_combo_box_text_new();
    struct { const char *id; const char *label; } tones[] = {
        {"220.0", "A3 (220 Hz)"}, {"261.63", "C4 (261 Hz)"}, {"329.63", "E4 (330 Hz)"},
        {"392.0", "G4 (392 Hz)"}, {"440.0", "A4 (440 Hz)"}, {"523.25", "C5 (523 Hz)"},
        {"659.25", "E5 (659 Hz)"}, {"880.0", "A5 (880 Hz)"}
    };
    for (unsigned int i = 0; i < sizeof(tones)/sizeof(tones[0]); ++i) {
        gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(ui->sample_preset), tones[i].id, tones[i].label);
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->sample_preset), 4);
    gtk_widget_set_halign(ui->sample_preset, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(sample_box), ui->sample_preset, FALSE, FALSE, 0);

    GtkWidget *dur_label = gtk_label_new("Sample-Länge (s)");
    gtk_widget_set_halign(dur_label, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(sample_box), dur_label, FALSE, FALSE, 0);
    ui->sample_duration = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.1, 5.0, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->sample_duration), e->sample_duration);
    g_signal_connect(ui->sample_duration, "value-changed", G_CALLBACK(on_sample_duration), e);
    gtk_widget_set_hexpand(ui->sample_duration, TRUE);
    gtk_box_pack_start(GTK_BOX(sample_box), ui->sample_duration, FALSE, FALSE, 0);

    GtkWidget *gain_label = gtk_label_new("Sample-Gain");
    gtk_widget_set_halign(gain_label, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(sample_box), gain_label, FALSE, FALSE, 0);
    ui->sample_gain = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.0, 1.5, 0.05);
    gtk_range_set_value(GTK_RANGE(ui->sample_gain), e->sample_gain);
    g_signal_connect(ui->sample_gain, "value-changed", G_CALLBACK(on_sample_gain), e);
    gtk_widget_set_hexpand(ui->sample_gain, TRUE);
    gtk_box_pack_start(GTK_BOX(sample_box), ui->sample_gain, FALSE, FALSE, 0);

    ui->sample_play = gtk_button_new_with_label("Sample abspielen");
    gtk_widget_set_halign(ui->sample_play, GTK_ALIGN_START);
    g_signal_connect(ui->sample_play, "clicked", G_CALLBACK(on_sample_play), ui);
    gtk_box_pack_start(GTK_BOX(sample_box), ui->sample_play, FALSE, FALSE, 0);

    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), grid, gtk_label_new("Decks"));
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), sample_box, gtk_label_new("Sampler"));

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
    eng.sample_duration = 1.0;
    eng.sample_gain = 0.6f;
    for (int d = 0; d < 2; ++d) {
        for (int i = 0; i < 10; ++i) eng.deck_eq_bands[d][i] = 1.0f;
    }
    for (int i = 0; i < 10; ++i) eng.master_eq_bands[i] = 1.0f;
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
        refresh_all_eq(&eng);
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
