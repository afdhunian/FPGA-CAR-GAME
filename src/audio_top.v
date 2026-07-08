// audio_top.v
//
// Top-level audio subsystem for the racing game. Drop this into
// car_game.v alongside render_static -- it needs the same collision/
// game_over/score signals render_static already produces.
//
// Wiring into car_game.v:
//
//   audio_top u_audio (
//       .clk         (CLOCK_50),     // board's 50MHz oscillator
//       .reset       (~KEY[0]),      // same reset as render_static
//       .collision   (collision),    // from render_static
//       .game_over   (game_over),    // from render_static
//       .score       (score),        // from render_static
//       .AUD_XCK     (AUD_XCK),
//       .AUD_BCLK    (AUD_BCLK),
//       .AUD_DACLRCK (AUD_DACLRCK),
//       .AUD_DACDAT  (AUD_DACDAT),
//       .I2C_SCLK    (FPGA_I2C_SCLK),
//       .I2C_SDAT    (FPGA_I2C_SDAT)
//   );
//
// Double check the exact AUD_*/I2C_* pin names against your board's
// pin planner / Quartus-generated top-level template -- Terasic keeps
// these names consistent across the DE-series but it's worth a quick
// visual check before compiling.

module audio_top (
    input  wire        clk,
    input  wire        reset,

    input  wire        collision,   // 1-cycle pulse from render_static
    input  wire        game_over,   // level signal from render_static
    input  wire [15:0] score,       // from render_static -- kept in the port list for wiring
                                     // compatibility with car_game.v, but unused now that
                                     // engine_sound is gone (nothing reads it anymore)

    output wire        AUD_XCK,
    output wire        AUD_BCLK,
    output wire        AUD_DACLRCK,
    output wire        AUD_DACDAT,
    output wire        I2C_SCLK,
    inout  wire        I2C_SDAT
);

    // ------------------------------------------------------------------
    // Master clock to the codec. See the note in i2s_transmitter.v --
    // this is a simple divider, not an exact 12.288MHz PLL output, which
    // is a fine simplification for game sound effects.
    // ------------------------------------------------------------------
    reg [3:0] xck_div;
    reg       xck_reg;
    always @(posedge clk) begin
        if (reset) begin
            xck_div <= 4'd0;
            xck_reg <= 1'b0;
        end else if (xck_div == 4'd7) begin
            xck_div <= 4'd0;
            xck_reg <= ~xck_reg;
        end else begin
            xck_div <= xck_div + 4'd1;
        end
    end
    assign AUD_XCK = xck_reg;

    // ------------------------------------------------------------------
    // I2C configuration (runs once after reset)
    // ------------------------------------------------------------------
    wire config_done;
    i2c_av_config u_i2c_config (
        .clk         (clk),
        .reset       (reset),
        .i2c_sclk    (I2C_SCLK),
        .i2c_sdat    (I2C_SDAT),
        .config_done (config_done)
    );

    // ------------------------------------------------------------------
    // The two sound sources (engine_sound removed)
    // ------------------------------------------------------------------
    wire signed [15:0] music_sample;
    music_rom_player u_music (
        .clk(clk), .reset(reset),
        .enable(!game_over && config_done),
        .sample(music_sample)
    );

    reg game_over_prev;
    always @(posedge clk) game_over_prev <= game_over;

    wire signed [15:0] crash_sample;
    wire               crash_active;
    crash_sfx u_crash (
        .clk(clk), .reset(reset),
        .trigger(collision || (game_over && !game_over_prev)), // fires on the hit itself; the OR is just a safety net in case game_over ever gets set from somewhere else without a collision pulse
        .sample(crash_sample),
        .active(crash_active)
    );

    // ------------------------------------------------------------------
    // Mixer: music normally; while the crash sound is playing, duck the
    // music so the impact actually cuts through instead of getting
    // buried, then simple saturation to stay within 16-bit range.
    // ------------------------------------------------------------------
    wire signed [15:0] music_ducked = crash_active ? (music_sample >>> 2) : music_sample;

    wire signed [16:0] mix_sum = music_ducked + crash_sample;

    wire signed [15:0] mix_final = (mix_sum >  17'sd32767)  ? 16'sd32767  :
                                    (mix_sum < -17'sd32768)  ? -16'sd32768 :
                                                                mix_sum[15:0];

    // ------------------------------------------------------------------
    // I2S output (mono mix duplicated to both channels)
    // ------------------------------------------------------------------
    wire sample_tick; // unused here, but available if you want to sync anything to the audio frame rate

    i2s_transmitter u_i2s (
        .clk          (clk),
        .reset        (reset),
        .sample_left  (mix_final),
        .sample_right (mix_final),
        .aud_bclk     (AUD_BCLK),
        .aud_daclrck  (AUD_DACLRCK),
        .aud_dacdat   (AUD_DACDAT),
        .sample_tick  (sample_tick)
    );

endmodule
