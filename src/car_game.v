module car_game (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    input  wire [9:0] SW,

    inout  wire       PS2_CLK,   // PS/2 keyboard port -- steering input
    inout  wire       PS2_DAT,

    output wire       VGA_CLK,
    output wire       VGA_HS,
    output wire       VGA_VS,
    output wire       VGA_BLANK_N,
    output wire       VGA_SYNC_N,
    output wire [7:0] VGA_R,
    output wire [7:0] VGA_G,
    output wire [7:0] VGA_B,

    output wire       AUD_XCK,
    output wire       AUD_BCLK,
    output wire       AUD_DACLRCK,
    output wire       AUD_DACDAT,
    output wire       FPGA_I2C_SCLK,
    inout  wire       FPGA_I2C_SDAT
);

    reg pixel_clk = 1'b0;

    always @(posedge CLOCK_50) begin
        pixel_clk <= ~pixel_clk;
    end

    wire reset;
    wire video_on;
    wire [9:0] x;
    wire [9:0] y;

    wire [7:0] red;
    wire [7:0] green;
    wire [7:0] blue;

    wire        collision;
    wire [15:0] score;      // HARUS 16-bit eksplisit -- kalau tidak, Verilog
                             // otomatis anggap 1-bit dan skor jadi rusak
    wire        game_over;

    assign reset = SW[9];

    wire steer_left, steer_right;
    ps2_steering u_kbd (
        .clk         (CLOCK_50),
        .reset       (~KEY[0]),
        .PS2_CLK     (PS2_CLK),
        .PS2_DAT     (PS2_DAT),
        .steer_left  (steer_left),
        .steer_right (steer_right)
    );

    vga_640x480 vga_unit (
        .pixel_clk(pixel_clk),
        .reset(reset),
        .x(x),
        .y(y),
        .video_on(video_on),
        .hsync(VGA_HS),
        .vsync(VGA_VS)
    );

    render_static render_unit (
	.clk(pixel_clk),        
	.video_on(video_on),
        .x(x),
        .y(y),
		  .steer_left(steer_left | ~KEY[2]),   // keyboard OR button -- either works
		  .steer_right(steer_right | ~KEY[1]), // keyboard OR button -- either works
		  .reset(~KEY[0]),
		  .collision(collision),
        .score(score),
		  .game_over(game_over),
        .red(red),
        .green(green),
        .blue(blue)
    );
	audio_top u_audio (
    .clk         (CLOCK_50),
    .reset       (~KEY[0]),
    .collision   (collision),
    .game_over   (game_over),
    .score       (score),
    .AUD_XCK     (AUD_XCK),
    .AUD_BCLK    (AUD_BCLK),
    .AUD_DACLRCK (AUD_DACLRCK),
    .AUD_DACDAT  (AUD_DACDAT),
    .I2C_SCLK    (FPGA_I2C_SCLK),
    .I2C_SDAT    (FPGA_I2C_SDAT)
);
    assign VGA_CLK     = pixel_clk;
    assign VGA_BLANK_N = video_on;
    assign VGA_SYNC_N  = 1'b1;

    assign VGA_R = red;
    assign VGA_G = green;
    assign VGA_B = blue;

endmodule
