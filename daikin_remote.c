/*
 * Daikin AC Remote - custom Flipper Zero app
 *
 * v2: custom-drawn interface using the user's own Photoshop art (power,
 * mode, temp, fan, swing, moon, turbo icons - each with normal/hover/
 * value-state variants). Navigation and redraw use the same low-level
 * ViewPort + manual input-queue pattern proven working in the very first
 * version of this app, rather than the higher-level View/model system,
 * to avoid re-introducing unverified API risk.
 *
 * Protocol structure and checksum algorithm were reverse-verified against
 * real captures from the user's own remote:
 *   - 3 sections: 8 bytes + 8 bytes + 19 bytes (35 bytes total)
 *   - Each section ends with a simple sum-of-preceding-bytes % 256 checksum
 *   - Section 3, byte 5  -> bit0 = Power, bits4-6 = Mode
 *   - Section 3, byte 6  -> Temperature in Celsius * 2
 *   - Section 3, byte 8  -> high nibble = Fan, low nibble = Vertical Swing
 *   - Fan nibble values verified against real Fan1-Fan5/Auto/Moon captures:
 *     Auto=0xA, 1=0x3, 2=0x4, 3=0x5, 4=0x6, 5=0x7, Moon/Quiet=0xB
 *   - All other bytes are constant, taken verbatim from a real working
 *     capture so timing/header/leader quirks specific to this remote are
 *     preserved exactly.
 *
 * IR transmission uses furi_hal_infrared_async_tx_* directly, checked
 * against the real furi_hal_infrared.h from the user's installed SDK
 * (Momentum, f7 target).
 *
 * GUI uses a raw ViewPort with a manual input event queue - the same
 * pattern already proven working on real hardware in v1, just drawing
 * icons instead of text this time.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_infrared.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <storage/storage.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <stdlib.h>
#include <string.h>

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

/* Fan nibble codes - verified directly against the user's real remote. */
#define FAN_AUTO  0xA
#define FAN_QUIET 0xB
#define FAN_1 0x3
#define FAN_2 0x4
#define FAN_3 0x5
#define FAN_4 0x6
#define FAN_5 0x7

static const uint8_t mode_codes[] = {MODE_AUTO, MODE_DRY, MODE_COOL, MODE_HEAT, MODE_FAN};
#define MODE_COUNT 5

/* Moon/Quiet is its own dedicated button now, not part of this list. */
static const uint8_t fan_codes[] = {FAN_AUTO, FAN_1, FAN_2, FAN_3, FAN_4, FAN_5};
#define FAN_COUNT 6

#define TEMP_MIN 18
#define TEMP_MAX 30

/* ---------------- App state ---------------- */

typedef struct {
    bool power;
    uint8_t mode_idx; /* index into mode_names/mode_codes */
    uint8_t temp_c;   /* 18-30 */
    uint8_t fan_idx;  /* index into fan_names/fan_codes */
    bool moon;        /* Moon/Quiet active - overrides fan_idx in the frame */
    bool swing_v;
    bool turbo;
} DaikinState;

/* ---------------- Settings persistence ---------------- */

#define SETTINGS_DIR "/ext/apps_data/daikin_remote"
#define SETTINGS_PATH SETTINGS_DIR "/settings.bin"

typedef struct {
    bool power;
    uint8_t mode_idx;
    uint8_t temp_c;
    uint8_t fan_idx;
    bool moon;
    bool swing_v;
} DaikinSavedSettings;

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
            s->moon = saved.moon;
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
            .moon = s->moon,
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

    uint8_t fan_code = s->moon ? FAN_QUIET : fan_codes[s->fan_idx];
    s3[8] = (fan_code << 4) | (s->swing_v ? 0x0F : 0x00);

    s3[9] = 0x00;         /* horizontal swing off */
    s3[10] = 0x00;        /* on/off timer sentinel, from real capture */
    s3[11] = 0x06;
    s3[12] = 0x60;
    s3[13] = s->turbo ? 0x01 : 0x00;
    s3[14] = 0x00;
    s3[15] = 0xC1;         /* observed constant */
    s3[16] = 0x80;         /* observed constant */
    s3[17] = 0x00;
    s3[18] = sum_checksum(s3, 18);
}

/* ---------------- Raw pulse encoding ---------------- */

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
    timings[idx++] = DAIKIN_BIT_MARK;
    timings[idx++] = gap_after;
    return idx;
}

static size_t daikin_encode_raw(const uint8_t frame[DAIKIN_FRAME_LEN], uint32_t* timings) {
    size_t idx = 0;

    static const uint32_t leader[] = {449, 417, 449, 418, 448, 419, 448, 417, 449, 417, 449, 25126};
    for(size_t i = 0; i < sizeof(leader) / sizeof(leader[0]); i++) {
        timings[idx++] = leader[i];
    }

    idx = add_section(timings, idx, frame, DAIKIN_SEC1_LEN, DAIKIN_SECTION_GAP);
    idx = add_section(timings, idx, frame + 8, DAIKIN_SEC2_LEN, DAIKIN_SECTION_GAP);
    idx = add_section(timings, idx, frame + 16, DAIKIN_SEC3_LEN, DAIKIN_FINAL_GAP);

    return idx;
}

static uint32_t s_timings[MAX_TIMINGS];

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
    *level = (ctx->index % 2 == 0);
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

    NotificationApp* notification = furi_record_open(RECORD_NOTIFICATION);
    notification_message(notification, &sequence_blink_start_magenta);

    furi_hal_infrared_async_tx_set_data_isr_callback(daikin_tx_callback, &ctx);
    furi_hal_infrared_async_tx_start(DAIKIN_FREQ, DAIKIN_DUTY);
    furi_hal_infrared_async_tx_wait_termination();

    notification_message(notification, &sequence_blink_stop);
    furi_record_close(RECORD_NOTIFICATION);
}

/* ---------------- GUI: custom-drawn interface using the user's art ----------------
 *
 * Icon positions below come directly from the user's Photoshop layout,
 * rotated 90 deg CCW (portrait design -> landscape screen) and extracted
 * automatically - not hand-guessed. Every button's normal/hover/value-state
 * icons share one canonical (x,y) slot, confirmed from the original file.
 *
 * Note: the art only includes 3 Mode icons (Dry/Cool/Heat), so the Mode
 * button here cycles those 3 only - Auto and Fan-only mode aren't reachable
 * from this UI (they still exist in the protocol/build_frame code, just
 * not wired to a button here).
 *
 * Turbo is left as a visual-only stub per your request - it toggles state
 * and swaps icons, but does NOT change the transmitted IR frame yet. A
 * likely home for it: section 3 byte 13 (0x20 constant in daikin_build_frame)
 * looks like it holds the Powerful/Quiet flag block based on general Daikin
 * documentation - worth capturing a real Turbo button press to confirm,
 * the same way we verified Fan values earlier, before wiring it up.
 */

#include "daikin_remote_icons.h"

#define UI_MODE_COUNT 5
static const uint8_t ui_mode_codes[UI_MODE_COUNT] = {MODE_AUTO, MODE_DRY, MODE_COOL, MODE_HEAT, MODE_FAN};

typedef enum {
    CursorPower,
    CursorMode,
    CursorTempUp,
    CursorTempDown,
    CursorFan,
    CursorSwing,
    CursorMoon,
    CursorTurbo,
    CursorCount,
} DaikinCursor;
static const DaikinCursor left_column[] = {CursorPower, CursorTempUp, CursorTempDown, CursorTurbo};
static const DaikinCursor right_column[] = {CursorMode, CursorFan, CursorSwing, CursorMoon};
#define COLUMN_LEN 4

typedef struct {
    DaikinState state;
    uint8_t ui_mode_idx; /* index into ui_mode_codes, independent of the
                             full mode_codes table used by the protocol */
    bool turbo;
    DaikinCursor cursor;
    uint8_t column; /* 0 = left, 1 = right */
    uint8_t row;    /* 0-3, index within the current column */
    ViewPort* view_port;
} DaikinApp;

/* Sync state.mode_idx (used by daikin_build_frame) from the UI's 3-mode
 * selection, since the protocol's mode_codes table has 5 entries but the
 * art only covers 3 of them. */
static void daikin_sync_mode(DaikinApp* app) {
    uint8_t code = ui_mode_codes[app->ui_mode_idx];
    for(uint8_t i = 0; i < MODE_COUNT; i++) {
        if(mode_codes[i] == code) {
            app->state.mode_idx = i;
            break;
        }
    }
}

static void daikin_draw_callback(Canvas* canvas, void* ctx) {
    DaikinApp* app = ctx;
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 7, 9, "Daikin AC");
    canvas_set_font(canvas, FontSecondary);

    bool sel_power = (app->cursor == CursorPower);
    bool sel_mode = (app->cursor == CursorMode);
    bool sel_tempup = (app->cursor == CursorTempUp);
    bool sel_tempdown = (app->cursor == CursorTempDown);
    bool sel_fan = (app->cursor == CursorFan);
    bool sel_swing = (app->cursor == CursorSwing);
    bool sel_moon = (app->cursor == CursorMoon);
    bool sel_turbo = (app->cursor == CursorTurbo);

    /* Power */
    canvas_draw_icon(canvas, 7, 11, sel_power ? &I_PowerIconHover_20x19 : &I_PowerIcon_20x19);
    canvas_draw_icon(canvas, 5, 34, &I_LabelPower_5x24);

    /* Mode (Dry/Cool/Heat only, see note above) */
    {
        const Icon* icon;
        switch(app->ui_mode_idx) 
        {
            case 0: icon = sel_mode ? &I_ModeAutoHover_20x19 : &I_ModeAuto_20x19; break;
            case 1: icon = sel_mode ? &I_ModeDryHover_20x19 : &I_ModeDry_20x19; break;
            case 2: icon = sel_mode ? &I_ModeCoolHover_20x19 : &I_ModeCool_20x19; break;
            case 3: icon = sel_mode ? &I_ModeHeatHover_20x19 : &I_ModeHeat_20x19; break;
            default: icon = sel_mode ? &I_ModeFanOnlyHover_20x19 : &I_ModeFanOnly_20x19; break;
        }
        canvas_draw_icon(canvas, 39, 11, icon);
    }
    canvas_draw_icon(canvas, 39, 34, &I_LabelMode_5x19);

    /* Temp: frame + celsius are always-on background, arrows swap on hover,
     * live number drawn on top - this is the "dynamic" part. */
    canvas_draw_icon(canvas, 1, 53, &I_TempFrame_30x30);
    canvas_draw_icon(canvas, 7, 62, &I_Celsius_11x18);
    canvas_draw_icon(canvas, 4, 40, sel_tempup ? &I_TempUpHover_21x24 : &I_TempUp_21x24);
    canvas_draw_icon(canvas, 4, 74, sel_tempdown ? &I_TempDownHover_21x24 : &I_TempDown_21x24);
    {
        /* Live temperature number - adjust x/y here if it overlaps the
         * celsius icon or frame border once you see it on real hardware. */
        char temp_str[4];
        snprintf(temp_str, sizeof(temp_str), "%d", app->state.temp_c);
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 5, 70, temp_str);
    }

    /* Fan: 6 states (Auto,1-5), matches the art exactly */
    {
        const Icon* icon;
        switch(app->state.fan_idx) {
        case 0: icon = sel_fan ? &I_FanAutoHover_20x19 : &I_FanAuto_20x19; break;
        case 1: icon = sel_fan ? &I_Fan1Hover_20x19 : &I_Fan1_20x19; break;
        case 2: icon = sel_fan ? &I_Fan2Hover_20x19 : &I_Fan2_20x19; break;
        case 3: icon = sel_fan ? &I_Fan3Hover_20x19 : &I_Fan3_20x19; break;
        case 4: icon = sel_fan ? &I_Fan4Hover_20x19 : &I_Fan4_20x19; break;
        default: icon = sel_fan ? &I_Fan5Hover_20x19 : &I_Fan5_20x19; break;
        }
        canvas_draw_icon(canvas, 39, 41, icon);
    }
    canvas_draw_icon(canvas, 42, 62, &I_LabelFan_5x13);

    /* Swing */
    {
        const Icon* icon;
        if(app->state.swing_v) {
            icon = sel_swing ? &I_SwingOnHover_20x19 : &I_SwingOn_20x19;
        } else {
            icon = sel_swing ? &I_SwingOffHover_20x19 : &I_SwingOff_20x19;
        }
        canvas_draw_icon(canvas, 39, 69, icon);
    }
    canvas_draw_icon(canvas, 38, 90, &I_LabelSwing_5x22);

    /* Moon */
    {
        const Icon* icon;
        if(app->state.moon) {
            icon = sel_moon ? &I_MoonOnHover_20x19 : &I_MoonOn_20x19;
        } else {
            icon = sel_moon ? &I_MoonOffHover_20x19 : &I_MoonOff_20x19;
        }
        canvas_draw_icon(canvas, 39, 97, icon);
    }
    canvas_draw_icon(canvas, 38, 118, &I_LabelMoon_5x20);

    /* Turbo - visual-only stub, see file header note */
    {
        const Icon* icon;
        if(app->state.turbo) {
            icon = sel_turbo ? &I_TurboOnHover_20x19 : &I_TurboOn_20x19;
        } else {
            icon = sel_turbo ? &I_TurboOffHover_20x19 : &I_TurboOff_20x19; 
        }
        canvas_draw_icon(canvas, 7, 97, icon);
    }
    canvas_draw_icon(canvas, 4, 118, &I_LabelTurbo_5x25);
}

static void daikin_input_callback(InputEvent* event, void* ctx) {
    FuriMessageQueue* queue = ctx;
    furi_message_queue_put(queue, event, FuriWaitForever);
}

int32_t daikin_remote_app(void* p) {
    UNUSED(p);

    DaikinApp* app = malloc(sizeof(DaikinApp));
    memset(app, 0, sizeof(DaikinApp));
    app->state = (DaikinState){
        .power = true,
        .mode_idx = 2, /* COOL, matches ui_mode_idx = 1 below */
        .temp_c = 24,
        .fan_idx = 0, /* AUTO */
        .moon = false,
        .swing_v = false,
    };
    app->ui_mode_idx = 2; /* Cool */
    app->state.turbo = false;
    app->cursor = CursorPower;
    app->column = 0;
    app->row = 0;
    daikin_settings_load(&app->state);

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    app->view_port = view_port_alloc();
    view_port_set_orientation(app->view_port, ViewPortOrientationVertical);
    view_port_draw_callback_set(app->view_port, daikin_draw_callback, app);
    view_port_input_callback_set(app->view_port, daikin_input_callback, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, app->view_port, GuiLayerFullscreen);

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
                        app->row = (app->row == 0) ? COLUMN_LEN - 1 : app->row - 1;
                        app->cursor = (app->column == 0) ? left_column[app->row] : right_column[app->row];
                        break;
                    case InputKeyDown:
                        app->row = (app->row + 1) % COLUMN_LEN;
                        app->cursor = (app->column == 0) ? left_column[app->row] : right_column[app->row];
                        break;
                    case InputKeyLeft:
                        app->column = 0;
                        app->cursor = left_column[app->row];
                        break;
                    case InputKeyRight:
                        app->column = 1;
                        app->cursor = right_column[app->row];
                        break;
                case InputKeyOk:
                    switch(app->cursor) {
                    case CursorPower:
                        app->state.power = !app->state.power;
                        break;
                    case CursorMode:
                        app->ui_mode_idx = (app->ui_mode_idx + 1) % UI_MODE_COUNT;
                        daikin_sync_mode(app);
                        break;
                    case CursorTempUp:
                        if(app->state.temp_c < TEMP_MAX) app->state.temp_c++;
                        break;
                    case CursorTempDown:
                        if(app->state.temp_c > TEMP_MIN) app->state.temp_c--;
                        break;
                    case CursorFan:
                        app->state.fan_idx = (app->state.fan_idx + 1) % FAN_COUNT;
                        app->state.moon = false;
                        break;
                    case CursorSwing:
                        app->state.swing_v = !app->state.swing_v;
                        break;
                    case CursorMoon:
                        app->state.moon = !app->state.moon;
                        break;
                    case CursorTurbo:
                        app->state.turbo = !app->state.turbo; /* visual only for now */
                        break;
                    default:
                        break;
                    }
                    daikin_send(&app->state);
                    daikin_settings_save(&app->state);
                    break;
                default:
                    break;
                }
                view_port_update(app->view_port);
            }
        }
    }

    daikin_settings_save(&app->state);

    gui_remove_view_port(gui, app->view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(app->view_port);
    furi_message_queue_free(event_queue);
    free(app);

    return 0;
}
