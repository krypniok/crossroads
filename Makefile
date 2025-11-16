CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -pedantic -O2
PKG_CONFIG ?= pkg-config
SOUNDTOUCH_PKG ?= soundtouch
GTK_PKG ?= gtk+-3.0

# Probe whether pkg-config knows all three libraries; fall back to manual flags.
ifeq ($(shell $(PKG_CONFIG) --exists openal sndfile $(SOUNDTOUCH_PKG) && echo yes),yes)
    AUDIO_CFLAGS := $(shell $(PKG_CONFIG) --cflags openal sndfile $(SOUNDTOUCH_PKG))
    AUDIO_LIBS := $(shell $(PKG_CONFIG) --libs openal sndfile $(SOUNDTOUCH_PKG))
else
    AUDIO_CFLAGS :=
    AUDIO_LIBS := -lopenal -lsndfile -lsoundtouch
endif

GTK_CFLAGS := $(shell $(PKG_CONFIG) --cflags $(GTK_PKG))
GTK_LIBS   := $(shell $(PKG_CONFIG) --libs $(GTK_PKG))

LDLIBS ?= -lpthread -lm -ldl

TARGET := crossroads
SRC := crossroads.c

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(AUDIO_CFLAGS) $(GTK_CFLAGS) $< -o $@ $(AUDIO_LIBS) $(GTK_LIBS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
