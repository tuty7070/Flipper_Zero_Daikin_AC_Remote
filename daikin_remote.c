/*
 * Daikin AC Remote - custom Flipper Zero app
 *
 * Builds a full 35-byte Daikin "classic" A/C IR state frame from
 * live user-selected settings (power, mode, temperature, fan, swing),
 * exactly like the real remote does, and transmits it as raw IR.
 *
 * Protocol structure and checksum algorithm were reverse-verified
 * against real captures from the user's own remote:
 *   - 3 sections: 8 bytes + 8 bytes + 19 bytes (35 bytes total)
 *   - Each section ends with a simple sum-of-preceding-bytes % 256 checksum
 *   - Section 3, byte 5  -> bit0 = Power, bits4-6 = Mode
 *   - Section 3, byte 6  -> Temperature in Celsius * 2
 *   - Section 3, byte 8  -> high nibble = Fan, low nibble = Vertical Swing
 *   - All other bytes are constant, taken verbatim from a real working
 *     capture (Power_ON) so timing/header/leader quirks specific to this
 *     remote are preserved exactly.
 *
 * NOTE: This was written and structurally verified against the user's
 * decoded IR capture. The IR-transmission call sequence below has now
 * been checked directly against the real furi_hal_infrared.h from the
 * user's installed SDK (Momentum, f7 target) - the earlier crash was a
 * call-order bug (data callback must be registered before starting
 * transmission, per that header's own doc comment), now fixed.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_infrared.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>
#include <stdlib.h>
#include <string.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>

#define TAG "DaikinRemote"

/* ---------------- Protocol constants (verified from real capture) --------------- */

#define DAIKIN_FREQ         38000
#define DAIKIN_DUTY         0.33f

#define DAIKIN_HDR_MARK      3434
#define DAIKIN_HDR_SPACE     1767
#define DAIKIN_BIT_MARK       428
#define DAIKIN_ZERO_SPACE     428
#define DAIKIN_ONE_SPACE     1285
#define DAIKIN_SECTION_GAP  35530
#define DAIKIN_FINAL_GAP    29000

#define DAIKIN_FRAME_LEN       35
#define DAIKIN_SEC1_LEN         8
#define DAIKIN_SEC2_LEN         8
#define DAIKIN_SEC3_LEN        19

/* Mode codes (from real Daikin classic protocol) */
#define MODE_AUTO 0
#define MODE_DRY  2
#define MODE_COOL 3
#define MODE_HEAT 4
#define MODE_FAN  6

/* Fan nibble codes - verified directly against the user's real remote
 * (captured Fan1-Fan5, Auto, Moon buttons). This unit's numbered levels
 * use nibble values 3-7, NOT 1-5 as general Daikin protocol docs suggest -
 * that mismatch is what caused "1" and "2" to do nothing previously. */
#define FAN_AUTO  0xA
#define FAN_QUIET 0xB
#define FAN_1 0x3
#define FAN_2 0x4
#define FAN_3 0x5
#define FAN_4 0x6
#define FAN_5 0x7

static const char* const mode_names[] = {"AUTO", "DRY", "COOL", "HEAT", "FAN"};
static const uint8_t mode_codes[] = {MODE_AUTO, MODE_DRY, MODE_COOL, MODE_HEAT, MODE_FAN};
#define MODE_COUNT 5

static const char* const fan_names[] = {"AUTO", "1", "2", "3", "4", "5", "MOON"};
static const uint8_t fan_codes[] = {FAN_AUTO, FAN_1, FAN_2, FAN_3, FAN_4, FAN_5, FAN_QUIET};
#define FAN_COUNT 7

/* ---------------- App state ---------------- */

typedef struct {
    bool power;
    uint8_t mode_idx; /* index into mode_names/mode_codes */
    uint8_t temp_c;   /* 18-30 */
    uint8_t fan_idx;  /* index into fan_names/fan_codes */
    bool swing_v;
    uint8_t cursor; /* 0=power 1=mode 2=temp 3=fan 4=swing */
    bool sending;
} DaikinState;

#define FIELD_COUNT 5
#define TEMP_MIN 18
#define TEMP_MAX 30

/* ---------------- Settings persistence ---------------- */

#define SETTINGS_DIR "/ext/apps_data/daikin_remote"
#define SETTINGS_PATH SETTINGS_DIR "/settings.bin"

typedef struct {
    bool power;
    uint8_t mode_idx;
    uint8_t temp_c;
    uint8_t fan_idx;
    bool swing_v;
} DaikinSavedSettings;

/* Loads saved settings over the given state's defaults, if a save file
 * exists. Values outside valid ranges (e.g. from an older version of this
 * app with a different FAN_COUNT) are ignored so we don't end up with an
 * out-of-bounds index. */
static void daikin_settings_load(DaikinState* s) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    if(storage_file_open(file, SETTINGS_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        DaikinSavedSettings saved;
        if(storage_file_read(file, &saved, sizeof(saved)) == sizeof(saved)) {
            s->power = saved.power;
            if(saved.mode_idx < MODE_COUNT) s->mode_idx = saved.mode_idx;
            if(saved.temp_c >= TEMP_MIN && saved.temp_c <= TEMP_MAX) s->temp_c = saved.temp_c;
            if(saved.fan_idx < FAN_COUNT) s->fan_idx = saved.fan_idx;
            s->swing_v = saved.swing_v;
        }
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static void daikin_settings_save(const DaikinState* s) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage, SETTINGS_DIR);
    File* file = storage_file_alloc(storage);
    if(storage_file_open(file, SETTINGS_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        DaikinSavedSettings saved = {
            .power = s->power,
            .mode_idx = s->mode_idx,
            .temp_c = s->temp_c,
            .fan_idx = s->fan_idx,
            .swing_v = s->swing_v,
        };
        storage_file_write(file, &saved, sizeof(saved));
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

/* ---------------- Frame building ---------------- */

static uint8_t sum_checksum(const uint8_t* buf, size_t len) {
    uint16_t sum = 0;
    for(size_t i = 0; i < len; i++) sum += buf[i];
    return (uint8_t)(sum & 0xFF);
}

static void daikin_build_frame(const DaikinState* s, uint8_t out[DAIKIN_FRAME_LEN]) {
    /* Section 1 - constant header, verified from real capture */
    static const uint8_t sec1_template[DAIKIN_SEC1_LEN - 1] = {0x11, 0xDA, 0x27, 0x00, 0xC5, 0x10, 0x00};
    memcpy(out, sec1_template, DAIKIN_SEC1_LEN - 1);
    out[7] = sum_checksum(out, 7);

    /* Section 2 - constant (clock/day field, safe fixed value from real capture) */
    static const uint8_t sec2_template[DAIKIN_SEC2_LEN - 1] = {0x11, 0xDA, 0x27, 0x00, 0x42, 0xEB, 0x13};
    memcpy(out + 8, sec2_template, DAIKIN_SEC2_LEN - 1);
    out[15] = sum_checksum(out + 8, 7);

    /* Section 3 - the part that actually changes per button press */
    uint8_t* s3 = out + 16;
    s3[0] = 0x11;
    s3[1] = 0xDA;
    s3[2] = 0x27;
    s3[3] = 0x00;
    s3[4] = 0x00;

    uint8_t mode_code = mode_codes[s->mode_idx];
    s3[5] = (s->power ? 0x01 : 0x00) | (mode_code << 4);

    uint8_t temp_clamped = s->temp_c;
    if(temp_clamped < TEMP_MIN) temp_clamped = TEMP_MIN;
    if(temp_clamped > TEMP_MAX) temp_clamped = TEMP_MAX;
    s3[6] = temp_clamped * 2;

    s3[7] = 0x00;

    uint8_t fan_code = fan_codes[s->fan_idx];
    s3[8] = (fan_code << 4) | (s->swing_v ? 0x0F : 0x00);

    s3[9] = 0x00;         /* horizontal swing off */
    s3[10] = 0x00;        /* on/off timer sentinel, from real capture */
    s3[11] = 0x06;
    s3[12] = 0x60;
    s3[13] = 0x20;         /* observed constant (quiet/powerful flags block) */
    s3[14] = 0x00;
    s3[15] = 0xC1;         /* observed constant */
    s3[16] = 0x80;         /* observed constant */
    s3[17] = 0x00;
    s3[18] = sum_checksum(s3, 18);
}

/* ---------------- Raw pulse encoding ---------------- */

/* Max size: leader(12) + sec1(2+16*8+2=146... computed generously) -> use generous buffer */
#define MAX_TIMINGS 700

static size_t add_section(
    uint32_t* timings,
    size_t idx,
    const uint8_t* bytes,
    size_t byte_count,
    uint32_t gap_after) {
    timings[idx++] = DAIKIN_HDR_MARK;
    timings[idx++] = DAIKIN_HDR_SPACE;
    for(size_t i = 0; i < byte_count; i++) {
        uint8_t b = bytes[i];
        for(uint8_t bit = 0; bit < 8; bit++) {
            bool one = (b >> bit) & 0x01;
            timings[idx++] = DAIKIN_BIT_MARK;
            timings[idx++] = one ? DAIKIN_ONE_SPACE : DAIKIN_ZERO_SPACE;
        }
    }
    /* footer mark + trailing gap */
    timings[idx++] = DAIKIN_BIT_MARK;
    timings[idx++] = gap_after;
    return idx;
}

static size_t daikin_encode_raw(const uint8_t frame[DAIKIN_FRAME_LEN], uint32_t* timings) {
    size_t idx = 0;

    /* Leader: fixed dummy pulses observed in real capture, doesn't encode data */
    static const uint32_t leader[] = {449, 417, 449, 418, 448, 419, 448, 417, 449, 417, 449, 25126};
    for(size_t i = 0; i < sizeof(leader) / sizeof(leader[0]); i++) {
        timings[idx++] = leader[i];
    }

    idx = add_section(timings, idx, frame, DAIKIN_SEC1_LEN, DAIKIN_SECTION_GAP);
    idx = add_section(timings, idx, frame + 8, DAIKIN_SEC2_LEN, DAIKIN_SECTION_GAP);
    idx = add_section(timings, idx, frame + 16, DAIKIN_SEC3_LEN, DAIKIN_FINAL_GAP);

    return idx;
}

/* Static, not malloc'd: read by the IR ISR callback for the whole
 * transmission, so it must not go out of scope or be freed mid-send. */
static uint32_t s_timings[MAX_TIMINGS];

/* Low-level, documented HAL API for async IR TX. This is the same
 * mechanism infrared_worker itself uses internally, but calling it
 * directly avoids depending on infrared_worker's exact (and possibly
 * SDK-version-dependent) function signature, which is what caused the
 * earlier furi_check crash.
 *
 * The callback is invoked from ISR context, once per mark/space value,
 * and must return:
 *   FuriHalInfraredTxGetDataStateOk       - more data follows
 *   FuriHalInfraredTxGetDataStateLastDone - this was the last value
 * Durations alternate mark (level=true) / space (level=false), starting
 * with a mark, matching how daikin_encode_raw() builds the array.
 */
typedef struct {
    const uint32_t* durations;
    size_t count;
    size_t index;
} DaikinTxCtx;

static FuriHalInfraredTxGetDataState
    daikin_tx_callback(void* context, uint32_t* duration, bool* level) {
    DaikinTxCtx* ctx = context;
    if(ctx->index >= ctx->count) {
        *duration = 0;
        *level = false;
        return FuriHalInfraredTxGetDataStateDone;
    }
    *duration = ctx->durations[ctx->index];
    *level = (ctx->index % 2 == 0); /* even index = mark, odd = space */
    ctx->index++;
    if(ctx->index >= ctx->count) {
        return FuriHalInfraredTxGetDataStateLastDone;
    }
    return FuriHalInfraredTxGetDataStateOk;
}

static void daikin_send(const DaikinState* s) {
    uint8_t frame[DAIKIN_FRAME_LEN];
    daikin_build_frame(s, frame);

    size_t count = daikin_encode_raw(frame, s_timings);

    DaikinTxCtx ctx = {.durations = s_timings, .count = count, .index = 0};

    /* Callback MUST be registered before starting transmission - the header
     * is explicit about this ("has to be called before
     * furi_hal_infrared_async_tx_start()"). Doing it in the other order is
     * what caused the fatal-error crash: the hardware started requesting
     * data via interrupt before any callback existed to answer it. */
    NotificationApp* notification = furi_record_open(RECORD_NOTIFICATION);
    notification_message(notification, &sequence_blink_start_magenta);
    furi_hal_infrared_async_tx_set_data_isr_callback(daikin_tx_callback, &ctx);
    furi_hal_infrared_async_tx_start(DAIKIN_FREQ, DAIKIN_DUTY);
    furi_hal_infrared_async_tx_wait_termination();
    notification_message(notification, &sequence_blink_stop);
    furi_record_close(RECORD_NOTIFICATION);
}

/* ---------------- GUI ---------------- */

static void draw_callback(Canvas* canvas, void* ctx) {
    DaikinState* s = ctx;
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 10, "Daikin AC Remote");
    canvas_set_font(canvas, FontSecondary);

    char line[48];

    snprintf(line, sizeof(line), "%s Power: %s", s->cursor == 0 ? ">" : " ", s->power ? "ON" : "OFF");
    canvas_draw_str(canvas, 2, 22, line);

    snprintf(line, sizeof(line), "%s Mode:  %s", s->cursor == 1 ? ">" : " ", mode_names[s->mode_idx]);
    canvas_draw_str(canvas, 2, 32, line);

    snprintf(line, sizeof(line), "%s Temp:  %d C", s->cursor == 2 ? ">" : " ", s->temp_c);
    canvas_draw_str(canvas, 2, 42, line);

    snprintf(line, sizeof(line), "%s Fan:   %s", s->cursor == 3 ? ">" : " ", fan_names[s->fan_idx]);
    canvas_draw_str(canvas, 2, 52, line);

    snprintf(line, sizeof(line), "%s Swing: %s", s->cursor == 4 ? ">" : " ", s->swing_v ? "ON" : "OFF");
    canvas_draw_str(canvas, 2, 62, line);

    canvas_draw_str(canvas, 78, 22, "L/R:value");
    canvas_draw_str(canvas, 78, 32, "Up/Dn:field");
    canvas_draw_str(canvas, 78, 42, "OK:send");
    canvas_draw_str(canvas, 78, 52, "BACK:exit");
    if(s->sending) {
        canvas_draw_str(canvas, 78, 62, "Sending...");
    }
}

static void input_callback(InputEvent* event, void* ctx) {
    FuriMessageQueue* queue = ctx;
    furi_message_queue_put(queue, event, FuriWaitForever);
}

int32_t daikin_remote_app(void* p) {
    UNUSED(p);

    DaikinState state = {
        .power = true,
        .mode_idx = 2, /* COOL */
        .temp_c = 24,
        .fan_idx = 0, /* AUTO */
        .swing_v = false,
        .cursor = 0,
        .sending = false,
    };
    daikin_settings_load(&state);

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, &state);
    view_port_input_callback_set(view_port, input_callback, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    InputEvent event;
    bool running = true;
    while(running) {
        if(furi_message_queue_get(event_queue, &event, 100) == FuriStatusOk) {
            if(event.type == InputTypeShort || event.type == InputTypeRepeat) {
                switch(event.key) {
                case InputKeyBack:
                    running = false;
                    break;
                case InputKeyUp:
                    state.cursor = (state.cursor == 0) ? FIELD_COUNT - 1 : state.cursor - 1;
                    break;
                case InputKeyDown:
                    state.cursor = (state.cursor + 1) % FIELD_COUNT;
                    break;
                case InputKeyLeft:
                case InputKeyRight: {
                    int8_t dir = (event.key == InputKeyRight) ? 1 : -1;
                    switch(state.cursor) {
                    case 0:
                        state.power = !state.power;
                        break;
                    case 1:
                        state.mode_idx = (state.mode_idx + MODE_COUNT + dir) % MODE_COUNT;
                        break;
                    case 2: {
                        int16_t t = state.temp_c + dir;
                        if(t < TEMP_MIN) t = TEMP_MIN;
                        if(t > TEMP_MAX) t = TEMP_MAX;
                        state.temp_c = (uint8_t)t;
                        break;
                    }
                    case 3:
                        state.fan_idx = (state.fan_idx + FAN_COUNT + dir) % FAN_COUNT;
                        break;
                    case 4:
                        state.swing_v = !state.swing_v;
                        break;
                    }
                    break;
                }
                case InputKeyOk:
                    state.sending = true;
                    view_port_update(view_port);
                    daikin_send(&state);
                    state.sending = false;
                    break;
                default:
                    break;
                }
                view_port_update(view_port);
            }
        }
    }

    gui_remove_view_port(gui, view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(view_port);
    furi_message_queue_free(event_queue);

    daikin_settings_save(&state);

    return 0;
}
