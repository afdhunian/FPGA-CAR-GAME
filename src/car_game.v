module car_game (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    input  wire [9:0] SW,

    output wire       VGA_CLK,
    output wire       VGA_HS,
    output wire       VGA_VS,
    output wire       VGA_BLANK_N,
    output wire       VGA_SYNC_N,
    output wire [7:0] VGA_R,
    output wire [7:0] VGA_G,
    output wire [7:0] VGA_B
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

    assign reset = SW[9];

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
        .video_on(video_on),
        .x(x),
        .y(y),
        .red(red),
        .green(green),
        .blue(blue)
    );

    assign VGA_CLK     = pixel_clk;
    assign VGA_BLANK_N = video_on;
    assign VGA_SYNC_N  = 1'b0;

    assign VGA_R = red;
    assign VGA_G = green;
    assign VGA_B = blue;

endmodule