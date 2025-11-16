#define _POSIX_C_SOURCE 200809L

#include <AL/al.h>
#include <AL/alc.h>

#include <sndfile.h>

#include <gtk/gtk.h>

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>
#include <dlfcn.h>
#include <dirent.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#define STREAM_BUFFER_FRAMES 2048
#define STREAM_BUFFERS 4
#define SHORT_PATH 192

static volatile sig_atomic_t g_should_quit = 0;

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

static void on_sigint(int s) {
    (void)s;
    g_should_quit = 1;
}

static void copy_safe(char *dst, size_t dstSize, const char *src) {
    if (dstSize == 0) {
        return;
    }
    if (!src) src = "";
    size_t n = strlen(src);
    if (n >= dstSize) n = dstSize - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static int ensure_soundtouch(char *err, size_t errsz) {
    if (g_st.loaded) return 0;
    if (g_st.attempted) {
        if (err && errsz) copy_safe(err, errsz, g_st.error[0] ? g_st.error : "SoundTouch API not available");
        return -1;
    }
    g_st.attempted = 1;
    const char *candidates[] = {
        "libSoundTouchDll.so.1",
        "libSoundTouchDll.so",
        "libsoundtouchdll.so.1",
        "libsoundtouchdll.so",
        "libSoundTouch.so.1",
        "libSoundTouch.so",
        "libsoundtouch.so.1",
        "libsoundtouch.so",
        NULL
    };
    for (size_t i = 0; candidates[i]; ++i) {
        g_st.handle = dlopen(candidates[i], RTLD_LOCAL | RTLD_LAZY);
        if (g_st.handle) break;
    }
    if (!g_st.handle) {
        copy_safe(g_st.error, sizeof(g_st.error), "SoundTouch library not found");
        if (err && errsz) copy_safe(err, errsz, g_st.error);
        return -1;
    }
    #define LOAD_SYM(name) do {                                                      \
        dlerror();                                                                   \
        *(void**)(&g_st.name) = dlsym(g_st.handle, "soundtouch_" #name);             \
        const char *se = dlerror();                                                  \
        if (se || !g_st.name) {                                                      \
            snprintf(g_st.error, sizeof(g_st.error), "SoundTouch symbol %s missing", #name); \
            if (err && errsz) copy_safe(err, errsz, g_st.error);                     \
            dlclose(g_st.handle);                                                    \
            g_st.handle = NULL;                                                      \
            return -1;                                                               \
        }                                                                            \
    } while(0)
    LOAD_SYM(createInstance);
    LOAD_SYM(destroyInstance);
    LOAD_SYM(setSampleRate);
    LOAD_SYM(setChannels);
    LOAD_SYM(setPitch);
    LOAD_SYM(setTempo);
    LOAD_SYM(putSamples);
    LOAD_SYM(receiveSamples);
    LOAD_SYM(flush);
    #undef LOAD_SYM
    g_st.loaded = 1;
    return 0;
}

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
    int buffering;
    int eof_sent;
    int finished;
    int beatmatch;
    float target_bpm;
    float detected_bpm;
    float tempo_ratio;
    char path[PATH_MAX];
} Stream;

typedef enum {
    CMD_PLAY_INDEX,
    CMD_NEXT,
    CMD_STOP,
    CMD_QUIT,
    CMD_SET_CROSSFADE,
    CMD_SET_TARGET_BPM,
    CMD_SET_BEATMATCH,
    CMD_REFRESH_TEMPO
} CommandType;

typedef struct {
    CommandType type;
    int index;
    double fvalue;
} Command;

typedef struct {
    GAsyncQueue *queue;
    Playlist *playlist;
    int current;
    int standby;
    Stream streams[2];
    ALCdevice *dev;
    ALCcontext *ctx;
    double crossfade_sec;
    int beatmatch;
    float target_bpm;
    double fade_start;
    int fading;
} AudioState;

typedef struct {
    GtkWidget *window;
    GtkWidget *list;
    GtkWidget *play_button;
    GtkWidget *next_button;
    GtkWidget *crossfade_scale;
    GtkWidget *bpm_scale;
    GtkWidget *beatmatch_toggle;
    AudioState *audio;
} UIState;

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void apply_tempo(Stream *s, float target_bpm, int beatmatch) {
    if (!s->source) return;
    if (!beatmatch || s->detected_bpm <= 0 || !s->st) {
        /* No tempo change */
        return;
    }
    float ratio = s->detected_bpm > 0 ? target_bpm / s->detected_bpm : 1.0f;
    if (ratio < 0.5f) ratio = 0.5f;
    if (ratio > 2.0f) ratio = 2.0f;
    g_st.setTempo(s->st, ratio);
}

static void destroy_stream(Stream *s) {
    if (s->source) {
        alSourceStop(s->source);
        ALint queued = 0;
        alGetSourcei(s->source, AL_BUFFERS_QUEUED, &queued);
        while (queued-- > 0) {
            ALuint b = 0;
            alSourceUnqueueBuffers(s->source, 1, &b);
        }
        alDeleteSources(1, &s->source);
        alDeleteBuffers(STREAM_BUFFERS, s->buffers);
    }
    if (s->sf) {
        sf_close(s->sf);
    }
    if (s->st && g_st.loaded) {
        g_st.destroyInstance(s->st);
    }
    memset(s, 0, sizeof(*s));
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
    double sum = 0;
    float *energy = calloc(blocks, sizeof(float));
    if (!energy) { free(buf); return 120.f; }
    for (size_t i = 0; i < blocks; ++i) {
        double acc = 0;
        for (size_t j = 0; j < block; ++j) {
            size_t idx = (i * block + j) * (size_t)info->channels;
            double v = 0;
            for (int c = 0; c < info->channels; ++c) v += fabsf(buf[idx + c]);
            acc += v / info->channels;
        }
        energy[i] = (float)(acc / block);
        sum += energy[i];
    }
    double avg = sum / blocks;
    double thresh = avg * 1.3;
    size_t prev = SIZE_MAX;
    double bpm_sum = 0;
    size_t bpm_count = 0;
    for (size_t i = 1; i + 1 < blocks; ++i) {
        if (energy[i] > thresh && energy[i] > energy[i-1] && energy[i] >= energy[i+1]) {
            if (prev != SIZE_MAX) {
                size_t diff = i > prev ? i - prev : prev - i;
                if (diff > 0) {
                    double interval = (double)(diff * block) / info->samplerate;
                    if (interval > 0.25 && interval < 2.0) {
                        double bpm = 60.0 / interval;
                        if (bpm >= 60 && bpm <= 200) {
                            bpm_sum += bpm;
                            bpm_count++;
                        }
                    }
                }
            }
            prev = i;
        }
    }
    free(energy);
    free(buf);
    if (!bpm_count) return 120.f;
    return (float)(bpm_sum / bpm_count);
}

static int init_stream(Stream *s, const char *path, float target_bpm, int beatmatch, char *err, size_t errsz) {
    memset(s, 0, sizeof(*s));
    copy_safe(s->path, sizeof(s->path), path);
    s->beatmatch = beatmatch;
    s->target_bpm = target_bpm;
    s->sf = sf_open(path, SFM_READ, &s->info);
    if (!s->sf) {
        snprintf(err, errsz, "sndfile error: %s", sf_strerror(NULL));
        return -1;
    }
    fprintf(stderr, "Lade %s (rate=%d, ch=%d, format=0x%x)\n", path, s->info.samplerate, s->info.channels, s->info.format);
    alGenSources(1, &s->source);
    alGenBuffers(STREAM_BUFFERS, s->buffers);
    s->buf_index = 0;
    s->detected_bpm = 0;
    s->tempo_ratio = 1.0f;
    if (beatmatch) {
        if (ensure_soundtouch(err, errsz) != 0) return -1;
        s->detected_bpm = detect_bpm_quick(s->sf, &s->info);
        float target = target_bpm > 0 ? target_bpm : s->detected_bpm;
        if (target <= 0) target = 120.f;
        s->tempo_ratio = s->detected_bpm > 0 ? target / s->detected_bpm : 1.0f;
        if (s->tempo_ratio < 0.5f) s->tempo_ratio = 0.5f;
        if (s->tempo_ratio > 2.0f) s->tempo_ratio = 2.0f;
        s->st = g_st.createInstance();
        if (!s->st) { snprintf(err, errsz, "SoundTouch init failed"); return -1; }
        g_st.setSampleRate(s->st, (unsigned int)s->info.samplerate);
        g_st.setChannels(s->st, (unsigned int)s->info.channels);
        g_st.setPitch(s->st, 1.0f);
        g_st.setTempo(s->st, s->tempo_ratio);
        sf_seek(s->sf, 0, SEEK_SET);
    } else {
        sf_seek(s->sf, 0, SEEK_SET);
    }
    alSourcef(s->source, AL_GAIN, 1.0f);
    alSourcef(s->source, AL_PITCH, 1.0f);
    alSourcei(s->source, AL_LOOPING, AL_FALSE);
    return 0;
}

static void queue_more(Stream *s) {
    if (s->finished) return;
    ALint queued = 0;
    alGetSourcei(s->source, AL_BUFFERS_QUEUED, &queued);
    while (queued < STREAM_BUFFERS) {
        float processed[STREAM_BUFFER_FRAMES * 2];
        unsigned int got = 0;
        int attempts = 0;
        while (attempts < 8 && got == 0) {
            float floatBuf[STREAM_BUFFER_FRAMES * 2];
            sf_count_t read = sf_readf_float(s->sf, floatBuf, STREAM_BUFFER_FRAMES);
            if (read > 0) {
                fprintf(stderr, "Read %ld frames from %s\n", (long)read, s->path);
            } else {
                fprintf(stderr, "EOF or read error on %s\n", s->path);
            }
            if (read <= 0) {
                if (s->st) g_st.flush(s->st);
                s->eof_sent = 1;
            } else if (s->st) {
                g_st.putSamples(s->st, floatBuf, (unsigned int)read);
            } else {
                got = (unsigned int)read;
                memcpy(processed, floatBuf, (size_t)got * s->info.channels * sizeof(float));
                break;
            }
            if (s->st) {
                got = g_st.receiveSamples(s->st, processed, STREAM_BUFFER_FRAMES);
            }
            attempts++;
        }
        if (got == 0) {
            if (s->eof_sent) s->finished = 1;
            break;
        }
        fprintf(stderr, "Queue %u frames (got=%u)\n", (unsigned)got, got);
        int16_t pcm[STREAM_BUFFER_FRAMES * 2];
        size_t samples = (size_t)got * (size_t)s->info.channels;
        for (size_t i = 0; i < samples; ++i) {
            float v = processed[i];
            if (v > 1.f) v = 1.f;
            else if (v < -1.f) v = -1.f;
            pcm[i] = (int16_t)lrintf(v * 32767.f);
        }
        ALenum fmt = s->info.channels == 1 ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;
        ALuint buf = s->buffers[s->buf_index++ % STREAM_BUFFERS];
        alBufferData(buf, fmt, pcm, (ALsizei)(samples * sizeof(int16_t)), s->info.samplerate);
        alSourceQueueBuffers(s->source, 1, &buf);
        queued++;
    }
}

static void update_stream(Stream *s) {
    if (!s->source) return;
    ALint processed = 0;
    alGetSourcei(s->source, AL_BUFFERS_PROCESSED, &processed);
    while (processed-- > 0) {
        ALuint buf;
        alSourceUnqueueBuffers(s->source, 1, &buf);
    }
    queue_more(s);
    if (s->finished) {
        ALint q = 0;
        alGetSourcei(s->source, AL_BUFFERS_QUEUED, &q);
        if (q == 0) {
            alSourceStop(s->source);
        }
    }
}

static void start_stream(Stream *s) {
    /* Prefill at least two buffers before starting to reduce underruns/pop noise */
    queue_more(s);
    ALint queued = 0;
    alGetSourcei(s->source, AL_BUFFERS_QUEUED, &queued);
    while (queued < 2) {
        queue_more(s);
        alGetSourcei(s->source, AL_BUFFERS_QUEUED, &queued);
        if (queued == 0) break;
    }
    if (queued > 0) {
        alSourcePlay(s->source);
    } else {
        fprintf(stderr, "Cannot start stream: no buffers queued for %s\n", s->path);
    }
}

static gpointer audio_thread(gpointer data) {
    AudioState *a = data;
    double last = now_sec();
    while (!g_should_quit) {
        Command *cmd = g_async_queue_try_pop(a->queue);
        if (cmd) {
            switch (cmd->type) {
            case CMD_PLAY_INDEX: {
                int idx = cmd->index;
                if (idx >= 0 && (size_t)idx < a->playlist->count) {
                    destroy_stream(&a->streams[a->standby]);
                    char err[128];
                    if (init_stream(&a->streams[a->standby], a->playlist->items[idx], a->target_bpm, a->beatmatch, err, sizeof(err)) == 0) {
                        start_stream(&a->streams[a->standby]);
                        if (a->streams[a->current].source == 0) {
                            a->current = a->standby;
                            a->standby = (a->standby + 1) % 2;
                            alSourcef(a->streams[a->current].source, AL_GAIN, 1.f);
                        } else {
                            a->fade_start = now_sec();
                            a->fading = 1;
                            alSourcef(a->streams[a->standby].source, AL_GAIN, 0.f);
                            alSourcef(a->streams[a->current].source, AL_GAIN, 1.f);
                        }
                    }
                }
                break;
            }
            case CMD_NEXT: {
                int next = (a->current + 1) % (int)a->playlist->count;
                Command *c = g_new(Command, 1);
                c->type = CMD_PLAY_INDEX; c->index = next; c->fvalue = 0;
                g_async_queue_push(a->queue, c);
                break;
            }
            case CMD_SET_CROSSFADE:
                a->crossfade_sec = cmd->fvalue;
                break;
            case CMD_SET_TARGET_BPM:
                a->target_bpm = (float)cmd->fvalue;
                break;
            case CMD_SET_BEATMATCH:
                a->beatmatch = (int)cmd->fvalue;
                break;
            case CMD_REFRESH_TEMPO:
                a->target_bpm = (float)cmd->fvalue;
                for (int i = 0; i < 2; ++i) {
                    apply_tempo(&a->streams[i], a->target_bpm, a->beatmatch);
                }
                break;
            case CMD_STOP:
                destroy_stream(&a->streams[0]);
                destroy_stream(&a->streams[1]);
                break;
            case CMD_QUIT:
                g_free(cmd);
                return NULL;
            }
            g_free(cmd);
        }

        for (int i = 0; i < 2; ++i) update_stream(&a->streams[i]);

        if (a->fading) {
            double t = now_sec() - a->fade_start;
            float x = (float)(t / a->crossfade_sec);
            if (x > 1.f) x = 1.f;
            alSourcef(a->streams[a->current].source, AL_GAIN, 1.f - x);
            alSourcef(a->streams[a->standby].source, AL_GAIN, x);
            if (t >= a->crossfade_sec) {
                alSourceStop(a->streams[a->current].source);
                destroy_stream(&a->streams[a->current]);
                int tmp = a->current;
                a->current = a->standby;
                a->standby = tmp;
                a->fading = 0;
            }
        }

        double now = now_sec();
        double dt = now - last;
        if (dt < 0.02) g_usleep((useconds_t)((0.02 - dt) * 1e6));
        last = now;
    }
    return NULL;
}

static int playlist_load(const char *dirpath, Playlist *pl, char *err, size_t errsz) {
    memset(pl, 0, sizeof(*pl));
    DIR *dir = opendir(dirpath);
    if (!dir) { snprintf(err, errsz, "Cannot open directory %s", dirpath); return -1; }
    struct dirent *ent;
    while ((ent = readdir(dir))) {
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
    closedir(dir);
    if (!pl->count) { snprintf(err, errsz, "No mp3 files in %s", dirpath); return -1; }
    qsort(pl->items, pl->count, sizeof(char *), (int (*)(const void *, const void *))strcmp);
    return 0;
}

static void playlist_free(Playlist *pl) {
    for (size_t i = 0; i < pl->count; ++i) free(pl->items[i]);
    free(pl->items);
}

static void send_command(AudioState *a, CommandType t, int idx, double val) {
    Command *c = g_malloc(sizeof(Command));
    c->type = t; c->index = idx; c->fvalue = val;
    g_async_queue_push(a->queue, c);
}

static void on_play_clicked(GtkButton *btn, gpointer user_data) {
    (void)btn;
    UIState *ui = user_data;
    GtkListBoxRow *row = gtk_list_box_get_selected_row(GTK_LIST_BOX(ui->list));
    if (!row) return;
    int idx = gtk_list_box_row_get_index(row);
    send_command(ui->audio, CMD_PLAY_INDEX, idx, 0);
}

static void on_next_clicked(GtkButton *btn, gpointer user_data) {
    (void)btn;
    UIState *ui = user_data;
    send_command(ui->audio, CMD_NEXT, 0, 0);
}

static void on_crossfade_changed(GtkRange *scale, gpointer user_data) {
    UIState *ui = user_data;
    double val = gtk_range_get_value(scale);
    send_command(ui->audio, CMD_SET_CROSSFADE, 0, val);
}

static void on_bpm_changed(GtkRange *scale, gpointer user_data) {
    UIState *ui = user_data;
    double val = gtk_range_get_value(scale);
    send_command(ui->audio, CMD_SET_TARGET_BPM, 0, val);
    send_command(ui->audio, CMD_REFRESH_TEMPO, 0, val);
}

static void on_beatmatch_toggled(GtkToggleButton *btn, gpointer user_data) {
    UIState *ui = user_data;
    gboolean active = gtk_toggle_button_get_active(btn);
    send_command(ui->audio, CMD_SET_BEATMATCH, 0, active);
    send_command(ui->audio, CMD_REFRESH_TEMPO, 0, gtk_range_get_value(GTK_RANGE(ui->bpm_scale)));
}

static void build_ui(UIState *ui, Playlist *pl) {
    ui->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(ui->window), "Winamp-Style DJ");
    gtk_window_set_default_size(GTK_WINDOW(ui->window), 720, 480);
    g_signal_connect(ui->window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_container_add(GTK_CONTAINER(ui->window), paned);

    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll, 260, -1);
    ui->list = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(ui->list), GTK_SELECTION_SINGLE);
    for (size_t i = 0; i < pl->count; ++i) {
        GtkWidget *row = gtk_label_new(pl->items[i]);
        gtk_widget_set_halign(row, GTK_ALIGN_START);
        gtk_label_set_ellipsize(GTK_LABEL(row), PANGO_ELLIPSIZE_MIDDLE);
        gtk_list_box_insert(GTK_LIST_BOX(ui->list), row, -1);
    }
    gtk_container_add(GTK_CONTAINER(scroll), ui->list);
    gtk_list_box_select_row(GTK_LIST_BOX(ui->list), gtk_list_box_get_row_at_index(GTK_LIST_BOX(ui->list), 0));
    gtk_paned_pack1(GTK_PANED(paned), scroll, TRUE, FALSE);

    GtkWidget *controls_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);
    gtk_container_set_border_width(GTK_CONTAINER(controls_box), 8);
    gtk_paned_pack2(GTK_PANED(paned), controls_box, FALSE, FALSE);

    GtkWidget *buttons = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(controls_box), buttons, FALSE, FALSE, 0);

    ui->play_button = gtk_button_new_with_label("Play/Blend");
    g_signal_connect(ui->play_button, "clicked", G_CALLBACK(on_play_clicked), ui);
    gtk_box_pack_start(GTK_BOX(buttons), ui->play_button, TRUE, TRUE, 0);

    ui->next_button = gtk_button_new_with_label("Next");
    g_signal_connect(ui->next_button, "clicked", G_CALLBACK(on_next_clicked), ui);
    gtk_box_pack_start(GTK_BOX(buttons), ui->next_button, TRUE, TRUE, 0);

    GtkWidget *fade_label = gtk_label_new("Crossfade (s)");
    gtk_box_pack_start(GTK_BOX(controls_box), fade_label, FALSE, FALSE, 0);
    ui->crossfade_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 1.0, 20.0, 0.5);
    gtk_scale_set_value_pos(GTK_SCALE(ui->crossfade_scale), GTK_POS_RIGHT);
    gtk_range_set_value(GTK_RANGE(ui->crossfade_scale), 6.0);
    g_signal_connect(ui->crossfade_scale, "value-changed", G_CALLBACK(on_crossfade_changed), ui);
    gtk_box_pack_start(GTK_BOX(controls_box), ui->crossfade_scale, FALSE, FALSE, 0);

    GtkWidget *bpm_label = gtk_label_new("Target BPM");
    gtk_box_pack_start(GTK_BOX(controls_box), bpm_label, FALSE, FALSE, 0);
    ui->bpm_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 60.0, 200.0, 1.0);
    gtk_scale_set_value_pos(GTK_SCALE(ui->bpm_scale), GTK_POS_RIGHT);
    gtk_range_set_value(GTK_RANGE(ui->bpm_scale), 128.0);
    g_signal_connect(ui->bpm_scale, "value-changed", G_CALLBACK(on_bpm_changed), ui);
    gtk_box_pack_start(GTK_BOX(controls_box), ui->bpm_scale, FALSE, FALSE, 0);

    ui->beatmatch_toggle = gtk_check_button_new_with_label("Beatmatch (Pitch-Lock)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->beatmatch_toggle), FALSE);
    g_signal_connect(ui->beatmatch_toggle, "toggled", G_CALLBACK(on_beatmatch_toggled), ui);
    gtk_box_pack_start(GTK_BOX(controls_box), ui->beatmatch_toggle, FALSE, FALSE, 0);

    gtk_widget_show_all(ui->window);
}

int main(int argc, char **argv) {
    const char *dirpath = (argc >= 2) ? argv[1] : "mp3_library";
    signal(SIGINT, on_sigint);

    Playlist pl;
    char err[256];
    if (playlist_load(dirpath, &pl, err, sizeof(err)) != 0) {
        fprintf(stderr, "%s\n", err);
        return 1;
    }

    ALCdevice *dev = alcOpenDevice(NULL);
    if (!dev) { fprintf(stderr, "OpenAL device failed\n"); return 1; }
    ALCcontext *ctx = alcCreateContext(dev, NULL);
    if (!ctx || !alcMakeContextCurrent(ctx)) { fprintf(stderr, "OpenAL context failed\n"); return 1; }

    gtk_init(&argc, &argv);

    AudioState audio = {0};
    audio.playlist = &pl;
    audio.queue = g_async_queue_new();
    audio.current = 0;
    audio.standby = 1;
    audio.crossfade_sec = 6.0;
    audio.target_bpm = 128.f;
    audio.beatmatch = 0;
    audio.dev = dev;
    audio.ctx = ctx;

    Command *init = g_malloc(sizeof(Command));
    init->type = CMD_PLAY_INDEX;
    init->index = 0;
    init->fvalue = 0;
    g_async_queue_push(audio.queue, init);

    UIState ui = {0};
    ui.audio = &audio;
    build_ui(&ui, &pl);

    g_thread_new("audio", (GThreadFunc)audio_thread, &audio);

    gtk_main();

    Command *quit = g_malloc(sizeof(Command));
    quit->type = CMD_QUIT; quit->index = 0; quit->fvalue = 0;
    g_async_queue_push(audio.queue, quit);
    g_usleep(200000);

    destroy_stream(&audio.streams[0]);
    destroy_stream(&audio.streams[1]);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(ctx);
    alcCloseDevice(dev);
    playlist_free(&pl);
    return 0;
}
