module vga_640x480 (
    input  wire       pixel_clk,
    input  wire       reset,
    output reg [9:0]  x,
    output reg [9:0]  y,
    output wire       video_on,
    output wire       hsync,
    output wire       vsync
);

    localparam H_VISIBLE = 640;
    localparam H_FRONT   = 16;
    localparam H_SYNC    = 96;
    localparam H_BACK    = 48;
    localparam H_TOTAL   = 800;

    localparam V_VISIBLE = 480;
    localparam V_FRONT   = 10;
    localparam V_SYNC    = 2;
    localparam V_BACK    = 33;
    localparam V_TOTAL   = 525;

    always @(posedge pixel_clk) begin
        if (reset) begin
            x <= 10'd0;
            y <= 10'd0;
        end else begin
            if (x == H_TOTAL - 1) begin
                x <= 10'd0;

                if (y == V_TOTAL - 1)
                    y <= 10'd0;
                else
                    y <= y + 10'd1;
            end else begin
                x <= x + 10'd1;
            end
        end
    end

    assign video_on = (x < H_VISIBLE) && (y < V_VISIBLE);

    assign hsync = ~((x >= H_VISIBLE + H_FRONT) &&
                     (x <  H_VISIBLE + H_FRONT + H_SYNC));

    assign vsync = ~((y >= V_VISIBLE + V_FRONT) &&
                     (y <  V_VISIBLE + V_FRONT + V_SYNC));

endmodule